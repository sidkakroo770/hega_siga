#!/usr/bin/env python3

from __future__ import annotations

import os

# Must be set before Gazebo protobuf imports.
os.environ.setdefault(
    "PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION",
    "python",
)

import argparse
import json
import math
import sys
import threading
import time

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional


from corridor_handoff import panel_front_distance
from corridor_altitude import AltitudeController

import cv2
import numpy as np

from pymavlink import mavutil

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan


# ============================================================
# PROJECT PATHS
# ============================================================

HOME = Path.home()
MISSION_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(MISSION_ROOT))

APPROACH_ROOT = (
    HOME
    / "sae_mission2"
    / "approach"
)

CORRIDOR_ROOT = (
    HOME
    / "sae_mission2"
    / "corridor"
)

sys.path.insert(
    0,
    str(APPROACH_ROOT),
)

sys.path.insert(
    0,
    str(CORRIDOR_ROOT),
)


# ============================================================
# EXISTING PROJECT MODULES
# ============================================================

from autonomy.perception.hybrid_banner_detector import (
    HybridBannerDetector,
)

from native.common.types import (
    Attitude,
    BodyVelocity,
    MissionState,
    NativeScan,
    VehicleAction,
)

from native.mission_runner import (
    MissionRunnerConfig,
    NativeMissionRunner,
    VehiclePose,
)

from coverage_mission.config import Config as CoverageConfig
from coverage_mission.runtime import main as run_coverage




# ============================================================
# EXPERIMENT FSM
# ============================================================

class ExperimentState(str, Enum):

    BANNER_SEARCH = "BANNER_SEARCH"

    CAMERA_CORRIDOR_CENTER = (
        "CAMERA_CORRIDOR_CENTER"
    )

    APPROACH_CORRIDOR = (
        "APPROACH_CORRIDOR"
    )

    HOVER_BEFORE_PRE_ENTRY = "HOVER_BEFORE_PRE_ENTRY"

    DESCEND_BEFORE_PRE_ENTRY = "DESCEND_BEFORE_PRE_ENTRY"

    LIDAR_CORRIDOR = (
        "LIDAR_CORRIDOR"
    )

    ASCEND_FOR_COVERAGE = "ASCEND_FOR_COVERAGE"

    ADVANCE_TO_FIELD = "ADVANCE_TO_FIELD"

    COVERAGE = "COVERAGE"

    COMPLETE = (
        "EXPERIMENT_COMPLETE"
    )

    ABORT = (
        "EXPERIMENT_ABORT"
    )


# ============================================================
# GAZEBO SENSOR CACHE
# ============================================================

sensor_lock = threading.Lock()

latest_forward_frame: Optional[
    np.ndarray
] = None

latest_scan: Optional[
    NativeScan
] = None

latest_lidar_world_pose = None

camera_sequence = 0
scan_sequence = 0


# ============================================================
# CAMERA CALLBACK
# ============================================================

def on_forward_image(
    msg: Image,
) -> None:

    global latest_forward_frame
    global camera_sequence

    try:

        image = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        image = image.reshape(
            (
                msg.height,
                msg.width,
                3,
            )
        )

        image = cv2.cvtColor(
            image,
            cv2.COLOR_RGB2BGR,
        )

    except Exception as exc:

        print(
            "[CAMERA] decode error:",
            exc,
        )

        return

    with sensor_lock:

        latest_forward_frame = image

        camera_sequence += 1


# ============================================================
# LIDAR CALLBACK
# ============================================================

def on_lidar(
    msg: LaserScan,
) -> None:

    global latest_scan
    global latest_lidar_world_pose
    global scan_sequence

    ranges = np.asarray(
        msg.ranges,
        dtype=np.float64,
    )

    if ranges.size == 0:
        return

    angles = (
        float(msg.angle_min)
        +
        np.arange(
            ranges.size,
            dtype=np.float64,
        )
        *
        float(msg.angle_step)
    )

    try:

        intensities = np.asarray(
            msg.intensities,
            dtype=np.float64,
        )

    except Exception:

        intensities = np.empty(0)

    if (
        intensities.size
        != ranges.size
    ):

        intensities = np.zeros(
            ranges.size,
            dtype=np.float64,
        )

    scan = NativeScan(
        angles_rad=angles,
        ranges_m=ranges,
        intensities=intensities,
        timestamp=time.monotonic(),
        range_min_m=float(
            msg.range_min
        ),
        range_max_m=float(
            msg.range_max
        ),
    )

    with sensor_lock:

        latest_scan = scan
        pose = msg.world_pose
        latest_lidar_world_pose = {
            "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
            "qx": pose.orientation.x, "qy": pose.orientation.y,
            "qz": pose.orientation.z, "qw": pose.orientation.w,
        }

        scan_sequence += 1


# ============================================================
# MAVLINK TELEMETRY CACHE
# ============================================================

@dataclass
class Telemetry:

    z_m: Optional[float] = None
    vz_m_s: Optional[float] = None
    vx_m_s: Optional[float] = None
    vy_m_s: Optional[float] = None
    x_m: Optional[float] = None
    y_m: Optional[float] = None

    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None

    position_boot_s: float = 0.0
    position_time: float = 0.0
    attitude_time: float = 0.0
    position_messages: int = 0
    attitude_messages: int = 0
    relative_alt_m: Optional[float] = None
    relative_alt_time: float = 0.0


telemetry = Telemetry()


def coverage_entry_registered(pose_telemetry, cfg, now):
    """Guard the fixed Gazebo field before advancing clear of the roof."""
    n, e = pose_telemetry.x_m, pose_telemetry.y_m
    return bool(
        n is not None and e is not None
        and math.isfinite(n) and math.isfinite(e)
        and cfg.n_min + cfg.body_radius <= n <= cfg.n_max - cfg.clearance
        and cfg.e_min + cfg.clearance <= e <= cfg.e_max - cfg.clearance
        and 0 <= now - pose_telemetry.position_time <= LIDAR_POSE_MAX_AGE_S
    )

# The full simulator reaches the manager through MAVProxy's UDP fan-out.  Its
# LOCAL_POSITION_NED packets normally arrive at 4 Hz unless explicitly raised,
# so one delayed packet must not invalidate a known pose mid-entry.
LIDAR_POSE_MAX_AGE_S = 1.0

# A routed MAVLink link can legitimately deliver pose packets around every
# quarter-second.  Refresh its stream request only for a meaningful delay;
# the 1.0-second pose-validity guard below remains the flight safety limit.
POSE_TELEMETRY_REFRESH_AGE_S = 0.75


def drain_mavlink(
    master,
) -> None:

    while True:

        msg = master.recv_match(
            blocking=False,
        )

        if msg is None:
            break

        now = time.monotonic()

        kind = msg.get_type()

        if (
            kind
            == "LOCAL_POSITION_NED"
        ):

            telemetry.x_m = float(
                msg.x
            )

            telemetry.y_m = float(
                msg.y
            )

            telemetry.position_boot_s = float(msg.time_boot_ms) / 1000.0
            telemetry.z_m = float(msg.z)
            telemetry.vz_m_s = float(msg.vz)
            telemetry.vx_m_s = float(msg.vx)
            telemetry.vy_m_s = float(msg.vy)
            telemetry.position_time = now
            telemetry.position_messages += 1

        elif (
            kind
            == "ATTITUDE"
        ):

            telemetry.roll_rad = float(
                msg.roll
            )

            telemetry.pitch_rad = float(
                msg.pitch
            )

            telemetry.yaw_rad = float(
                msg.yaw
            )

            telemetry.attitude_time = now
            telemetry.attitude_messages += 1

        elif kind == "GLOBAL_POSITION_INT":
            telemetry.relative_alt_m = float(msg.relative_alt) * 0.001
            telemetry.relative_alt_time = now


# ============================================================
# VEHICLE POSE FOR NATIVE FSM
# ============================================================

def current_pose() -> Optional[
    VehiclePose
]:

    now = time.monotonic()

    if (
        telemetry.x_m is None
        or telemetry.y_m is None
        or telemetry.yaw_rad is None
    ):
        return None

    if (
        now - telemetry.position_time
        > LIDAR_POSE_MAX_AGE_S
    ):
        return None

    if (
        now - telemetry.attitude_time
        > LIDAR_POSE_MAX_AGE_S
    ):
        return None

    return VehiclePose(

        x_m=telemetry.x_m,

        y_m=telemetry.y_m,

        yaw_rad=telemetry.yaw_rad,

        # Position freshness is what ENTER_CORRIDOR
        # actually depends upon.
        timestamp=telemetry.position_time,
    )


# ============================================================
# ATTITUDE FOR NATIVE FSM
# ============================================================

def current_attitude() -> Optional[
    Attitude
]:

    now = time.monotonic()

    if (
        telemetry.roll_rad is None
        or telemetry.pitch_rad is None
        or telemetry.yaw_rad is None
    ):
        return None

    if (
        now - telemetry.attitude_time
        > 0.50
    ):
        return None

    return Attitude(

        roll_rad=telemetry.roll_rad,

        pitch_rad=telemetry.pitch_rad,

        yaw_rad=telemetry.yaw_rad,

        timestamp=(
            telemetry.attitude_time
        ),
    )


# ============================================================
# CAMERA MAVLINK CONTROL
# ============================================================

def send_camera_velocity(
    master,
    vx: float,
    vy: float,
    vz: float,
) -> None:

    """
    Same velocity packet style used by shhhh.

    MAV_FRAME_BODY_OFFSET_NED:

        +X = forward
        +Y = right
        +Z = down
    """

    type_mask = int(
        0b0000111111000111
    )

    master.mav.send(

        mavutil.mavlink.
        MAVLink_set_position_target_local_ned_message(

            10,

            master.target_system,
            master.target_component,

            mavutil.mavlink.
            MAV_FRAME_BODY_OFFSET_NED,

            type_mask,

            # position ignored
            0.0,
            0.0,
            0.0,

            # velocity
            float(vx),
            float(vy),
            float(vz),

            # acceleration ignored
            0.0,
            0.0,
            0.0,

            # yaw ignored
            0.0,

            # yaw-rate ignored
            0.0,
        )
    )


def send_stop(
    master,
) -> None:

    send_camera_velocity(
        master,
        0.0,
        0.0,
        0.0,
    )


# ============================================================
# NATIVE FSM MAVLINK CONTROL
# ============================================================

def send_native_velocity(
    master,
    command: BodyVelocity,
) -> None:

    """
    Native FSM uses FLU:

        +X = forward
        +Y = left
        +Z = up
        +yaw = CCW

    MAVLink BODY_NED:

        +X = forward
        +Y = right
        +Z = down
        +yaw = clockwise

    Therefore:

        Y        -> negate
        Z        -> negate
        yaw-rate -> negate

    Type mask 1479:
        ignore position
        USE velocity
        ignore acceleration
        ignore yaw angle
        USE yaw-rate
    """

    type_mask = 1479

    master.mav.set_position_target_local_ned_send(

        int(
            time.monotonic()
            * 1000
        )
        & 0xFFFFFFFF,

        master.target_system,
        master.target_component,

        mavutil.mavlink.
        MAV_FRAME_BODY_NED,

        type_mask,

        # position ignored
        0.0,
        0.0,
        0.0,

        # velocity
        float(
            command.vx_m_s
        ),

        float(
            -command.vy_m_s
        ),

        float(
            -command.vz_m_s
        ),

        # acceleration ignored
        0.0,
        0.0,
        0.0,

        # yaw angle ignored
        0.0,

        # yaw-rate ACTIVE
        float(
            -command.yaw_rate_rad_s
        ),
    )


# ============================================================
# LAND
# ============================================================

def request_land(
    master,
) -> None:

    print(
        "[SAFETY] LAND requested"
    )

    master.mav.command_long_send(

        master.target_system,
        master.target_component,

        mavutil.mavlink.
        MAV_CMD_NAV_LAND,

        0,

        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


# ============================================================
# TELEMETRY STREAM REQUESTS
# ============================================================

def request_message_interval(
    master,
    message_id: int,
    hz: float,
) -> None:

    interval_us = int(
        1_000_000
        /
        max(
            hz,
            0.1,
        )
    )

    master.mav.command_long_send(

        master.target_system,
        master.target_component,

        mavutil.mavlink.
        MAV_CMD_SET_MESSAGE_INTERVAL,

        0,

        message_id,
        interval_us,

        0,
        0,
        0,
        0,
        0,
    )


def request_pose_telemetry(
    master,
    hz: float = 20.0,
) -> None:
    """Request pose streams through both supported ArduPilot mechanisms.

    COMMAND_LONG SET_MESSAGE_INTERVAL is preferred.  REQUEST_DATA_STREAM is a
    compatibility fallback for a MAVProxy-routed SITL link that ignores or
    delays the newer request.  Neither request controls vehicle movement.
    """

    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        hz,
    )
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        hz,
    )
    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        hz,
    )
    stream_hz = max(1, int(round(hz)))
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        stream_hz,
        1,
    )
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        stream_hz,
        1,
    )


# ============================================================
# HELPERS
# ============================================================

def clamp(
    value: float,
    minimum: float,
    maximum: float,
) -> float:

    return max(
        minimum,
        min(
            maximum,
            value,
        ),
    )


def create_corridor_runner(enter_distance):
    """Longer stage deadlines for the slow local Gazebo simulation."""
    runner = NativeMissionRunner(config=MissionRunnerConfig(
        enter_corridor_distance_m=enter_distance,
        enter_corridor_max_pose_age_s=LIDAR_POSE_MAX_AGE_S,
        enter_corridor_pose_timeout_s=6.0,
        pre_entry_hold_timeout_s=32.0,
        # The routed full simulation can make less than the commanded 0.20
        # m/s while retaining valid position feedback.  Keep the measured
        # 0.75 m entry requirement and allow sufficient time to reach it.
        enter_corridor_timeout_s=60.0,
        reassess_hard_timeout_s=48.0,
    ))
    runner.pre_entry.config.acquire_timeout_s = 32.0
    runner.pre_entry.config.alignment_timeout_s = 60.0
    runner.reassess.config.recovery_timeout_s = 32.0
    # SITL boot time advances with simulation physics, as in descent/hover.
    # SIMULATION ONLY: review these overrides before actual aircraft testing.
    runner.exit.progress_clock = lambda: telemetry.position_boot_s
    runner.exit.config.pose_fresh_s = LIDAR_POSE_MAX_AGE_S
    runner.exit.config.pose_loss_timeout_s = 6.0
    runner.exit.config.wall_timeout_s = 180.0
    return runner


# ============================================================
# MAIN
# ============================================================

def main() -> int:

    parser = argparse.ArgumentParser()


    # --------------------------------------------------------
    # MAVLink
    # --------------------------------------------------------

    parser.add_argument(
        "--mavlink",

        default=(
            "udpin:0.0.0.0:14552"
        ),
    )


    # --------------------------------------------------------
    # Corridor entry
    # --------------------------------------------------------

    parser.add_argument(
        "--enter-distance",

        type=float,

        default=0.75,
    )


    # --------------------------------------------------------
    # Banner search
    # --------------------------------------------------------

    parser.add_argument(
        "--search-speed",

        type=float,

        default=0.50,
    )


    # --------------------------------------------------------
    # Camera control
    # --------------------------------------------------------

    parser.add_argument(
        "--camera-gain",

        type=float,

        default=0.003,
    )

    parser.add_argument(
        "--camera-max-speed",

        type=float,

        default=0.50,
    )

    parser.add_argument(
        "--center-tolerance-px",

        type=float,

        default=20.0,
    )

    parser.add_argument(
        "--center-frames",

        type=int,

        default=30,
    )


    # --------------------------------------------------------
    # NEW APPROACH STATE
    # --------------------------------------------------------

    parser.add_argument(
        "--approach-speed",

        type=float,

        default=0.20,

        help=(
            "Forward speed while camera remains "
            "centered until the range limit or confirmed panel loss."
        ),
    )

    parser.add_argument(
        "--banner-loss-frames",

        type=int,

        default=5,
    )

    parser.add_argument("--entrance-commit-range", type=float, default=0.5,
                        help="End camera approach at this forward LiDAR range in metres")
    parser.add_argument("--pre-entry-descent", type=float, default=1.0,
                        help="Metres to descend after the camera-to-LiDAR handoff")
    parser.add_argument("--coverage-config", type=Path,
                        default=MISSION_ROOT / "config/full_mission_coverage.json",
                        help="Registered full-world coverage field and camera profile")
    parser.add_argument("--coverage-log", type=Path,
                        default=MISSION_ROOT / "world/integration/artifacts/coverage_runtime.jsonl")
    parser.add_argument("--coverage-max-wall-seconds", type=float, default=3600)
    parser.add_argument(
        "--debug-output",
        type=Path,
        default=HOME / "sae_mission2/world/integration/artifacts/preentry_capture",
        help="Persistent directory for the first real PRE_ENTRY LiDAR capture",
    )
    args = parser.parse_args()
    if not math.isfinite(args.entrance_commit_range) or args.entrance_commit_range <= 0:
        parser.error("--entrance-commit-range must be finite and positive")
    if not math.isfinite(args.pre_entry_descent) or args.pre_entry_descent <= 0:
        parser.error("--pre-entry-descent must be finite and positive")
    if args.banner_loss_frames < 1:
        parser.error("--banner-loss-frames must be at least 1")
    if not math.isfinite(args.coverage_max_wall_seconds) or args.coverage_max_wall_seconds <= 0:
        parser.error("--coverage-max-wall-seconds must be finite and positive")
    coverage_cfg = CoverageConfig.load(args.coverage_config)



    # ========================================================
    # HEADER
    # ========================================================

    print()

    print(
        "======================================================"
    )

    print(
        " EXPERIMENTAL CAMERA -> LIDAR CORRIDOR FSM"
    )

    print(
        "======================================================"
    )

    print()

    print(
        "BANNER_SEARCH"
    )

    print(
        "      ↓"
    )

    print(
        "CAMERA_CORRIDOR_CENTER"
    )

    print(
        "      ↓"
    )

    print(
        f"APPROACH_CORRIDOR -> PANEL LOST / {args.entrance_commit_range:g} m RANGE -> "
        f"DESCEND {args.pre_entry_descent:g} m"
    )

    print(
        "      ↓"
    )

    print(
        "PRE_ENTRY_GEOMETRY_LOCK"
    )

    print(
        "      ↓"
    )

    print(
        "CORRIDOR FSM -> EXIT_DETECTION"
    )

    print("      ↓")
    print("ADVANCE_TO_FIELD -> ASCEND_FOR_COVERAGE -> COVERAGE -> COMPLETE")

    print()

    print(
        "APPROACH speed:",
        f"{args.approach_speed:.2f} m/s",
    )

    print(
        "ENTER_CORRIDOR distance:",
        f"{args.enter_distance:.2f} m",
    )

    print()


    # ========================================================
    # MAVLINK
    # ========================================================

    print(
        "[MAVLINK] connecting:",
        args.mavlink,
    )

    master = (
        mavutil.mavlink_connection(

            args.mavlink,

            source_system=254,
        )
    )

    master.wait_heartbeat()

    print(
        "[MAVLINK] heartbeat received:",
        f"sys={master.target_system}",
        f"comp={master.target_component}",
    )


    # Request faster pose streams before any movement starts.
    request_pose_telemetry(master)
    print("[MAVLINK] requested LOCAL_POSITION_NED and ATTITUDE at 20 Hz")


    # ========================================================
    # GAZEBO SUBSCRIPTIONS
    # ========================================================

    node = Node()

    camera_ok = node.subscribe(

        Image,

        "/iris/camera_forward/image_raw",

        on_forward_image,
    )

    lidar_ok = node.subscribe(

        LaserScan,

        "/iris/lidar/scan",

        on_lidar,
    )

    print(
        "[GAZEBO] forward camera:",
        camera_ok,
    )

    print(
        "[GAZEBO] lidar:",
        lidar_ok,
    )

    if not camera_ok:

        raise RuntimeError(
            "forward camera subscription failed"
        )

    if not lidar_ok:

        raise RuntimeError(
            "LiDAR subscription failed"
        )


    # ========================================================
    # CAMERA DETECTOR
    # ========================================================

    banner_detector = (
        HybridBannerDetector()
    )


    # ========================================================
    # STATE
    # ========================================================

    state = (
        ExperimentState.
        BANNER_SEARCH
    )


    # Actual corridor FSM.
    #
    # Created ONLY when LiDAR handoff happens.
    corridor: Optional[
        NativeMissionRunner
    ] = None


    centered_frames = 0

    banner_lost_frames = 0
    approach_detection_diag = "waiting_for_new_camera_frame"

    last_camera_sequence = -1
    last_scan_sequence = -1

    last_diag = 0.0

    previous_native_state = None
    descent = None
    descent_started = 0.0
    hover_stable_since = None
    hover_started = 0.0
    hover_boot_start = 0.0
    last_stream_retry = 0.0
    preentry_snapshot_saved = False
    coverage_climb = None
    coverage_started = False
    field_advance_started = 0.0
    field_advance_boot_start = 0.0


    print()

    print(
        "[EXPERIMENT] START -> "
        "BANNER_SEARCH"
    )

    print()


    # ========================================================
    # MAIN LOOP
    # ========================================================

    try:

        while True:

            loop_started = (
                time.monotonic()
            )

            drain_mavlink(
                master
            )

            now = (
                time.monotonic()
            )


            # Another GCS can overwrite the startup interval request. Retry
            # only when received telemetry is slow, with a bounded request rate.
            if now - last_stream_retry >= 2.0 and (
                now - telemetry.position_time > POSE_TELEMETRY_REFRESH_AGE_S
                or now - telemetry.attitude_time > POSE_TELEMETRY_REFRESH_AGE_S
            ):
                request_pose_telemetry(master)
                pos_age = now - telemetry.position_time
                att_age = now - telemetry.attitude_time
                print(f"[MAVLINK] retrying pose telemetry request: "
                      f"position_age={pos_age:.2f}s attitude_age={att_age:.2f}s")
                last_stream_retry = now

            # =================================================
            # STATE 1
            # BANNER SEARCH
            # =================================================

            if (
                state
                ==
                ExperimentState.
                BANNER_SEARCH
            ):

                with sensor_lock:

                    frame = (
                        None
                        if latest_forward_frame
                        is None
                        else
                        latest_forward_frame.copy()
                    )

                    cam_seq = (
                        camera_sequence
                    )


                if frame is None:

                    send_stop(
                        master
                    )

                    if (
                        now - last_diag
                        >= 1.0
                    ):

                        print(
                            "[BANNER_SEARCH] "
                            "waiting for forward camera"
                        )

                        last_diag = now

                    time.sleep(
                        0.02
                    )

                    continue


                if (
                    cam_seq
                    != last_camera_sequence
                ):

                    last_camera_sequence = (
                        cam_seq
                    )

                    result = (
                        banner_detector.detect(
                            frame
                        )
                    )


                    if not result[
                        "detected"
                    ]:

                        # Search LEFT.
                        #
                        # BODY_NED +Y = right,
                        # therefore negative Y = left.

                        send_camera_velocity(

                            master,

                            vx=0.0,

                            vy=(
                                -abs(
                                    args.search_speed
                                )
                            ),

                            vz=0.0,
                        )


                        if (
                            now - last_diag
                            >= 0.50
                        ):

                            print(
                                "[BANNER_SEARCH] "
                                "not detected "
                                "-> moving LEFT"
                            )

                            last_diag = now


                    else:

                        send_stop(
                            master
                        )

                        centered_frames = 0
                        banner_detector.reset_track()

                        state = (
                            ExperimentState.
                            CAMERA_CORRIDOR_CENTER
                        )


                        print()

                        print(
                            "=========================================="
                        )

                        print(
                            " GREEN BANNER DETECTED"
                        )

                        print(
                            " BANNER_SEARCH -> "
                            "CAMERA_CORRIDOR_CENTER"
                        )

                        print(
                            "=========================================="
                        )

                        print()


                    debug = (
                        result[
                            "debug_frame"
                        ]
                    )

                    cv2.putText(

                        debug,

                        "STATE: BANNER_SEARCH",

                        (
                            10,
                            145,
                        ),

                        cv2.FONT_HERSHEY_SIMPLEX,

                        0.55,

                        (
                            255,
                            255,
                            255,
                        ),

                        2,
                    )

                    cv2.imshow(
                        "Experimental Corridor Camera",
                        debug,
                    )


            # =================================================
            # STATE 2
            # CAMERA CENTER
            # =================================================

            elif (
                state
                ==
                ExperimentState.
                CAMERA_CORRIDOR_CENTER
            ):

                with sensor_lock:

                    frame = (
                        None
                        if latest_forward_frame
                        is None
                        else
                        latest_forward_frame.copy()
                    )

                    cam_seq = (
                        camera_sequence
                    )


                if frame is None:

                    send_stop(
                        master
                    )

                    continue


                if (
                    cam_seq
                    != last_camera_sequence
                ):

                    last_camera_sequence = (
                        cam_seq
                    )

                    result = (
                        banner_detector.detect(
                            frame
                        )
                    )


                    if not result[
                        "detected"
                    ]:

                        send_stop(
                            master
                        )

                        centered_frames = 0
                        banner_detector.reset_track()

                        state = (
                            ExperimentState.
                            BANNER_SEARCH
                        )

                        print(
                            "[CAMERA_CENTER] "
                            "banner lost "
                            "-> BANNER_SEARCH"
                        )


                    else:

                        error_x = float(
                            result[
                                "error_x"
                            ]
                        )

                        error_y = float(
                            result[
                                "error_y"
                            ]
                        )


                        # Target right -> move right.
                        vy = clamp(

                            error_x
                            *
                            args.camera_gain,

                            -args.camera_max_speed,

                            args.camera_max_speed,
                        )


                        # Target below -> move down.
                        vz = clamp(

                            error_y
                            *
                            args.camera_gain,

                            -args.camera_max_speed,

                            args.camera_max_speed,
                        )


                        send_camera_velocity(

                            master,

                            vx=0.0,

                            vy=vy,

                            vz=vz,
                        )


                        if (
                            abs(error_x)
                            <
                            args.center_tolerance_px
                            and
                            abs(error_y)
                            <
                            args.center_tolerance_px
                        ):

                            centered_frames += 1

                        else:

                            centered_frames = 0


                        if (
                            now - last_diag
                            >= 0.50
                        ):

                            print(

                                "[CAMERA_CENTER] "

                                f"ex="
                                f"{error_x:+.1f}px "

                                f"ey="
                                f"{error_y:+.1f}px "

                                f"stable="
                                f"{centered_frames}/"
                                f"{args.center_frames} "

                                f"| vy="
                                f"{vy:+.2f} "

                                f"vz="
                                f"{vz:+.2f}"
                            )

                            last_diag = now


                        # -------------------------------------
                        # CAMERA CENTER COMPLETE
                        # -------------------------------------

                        if (
                            centered_frames
                            >=
                            args.center_frames
                        ):

                            send_stop(
                                master
                            )


                            banner_lost_frames = 0
                            # Corridor is gray: allow partial green panel views
                            # without temporal size/shape/edge rejection.
                            banner_detector.panel_only = False
                            banner_detector.reset_track()

                            state = ExperimentState.APPROACH_CORRIDOR


                            print()

                            print(
                                "=========================================="
                            )

                            print(
                                " CAMERA CENTER COMPLETE"
                            )

                            print(
                                " -> APPROACH_CORRIDOR"
                            )

                            print(
                                " Camera keeps alignment."
                            )

                            print(
                                " Panel loss stops approach and starts the LiDAR handoff."
                            )

                            print(
                                "=========================================="
                            )

                            print()


                    debug = (
                        result[
                            "debug_frame"
                        ]
                    )

                    cv2.putText(

                        debug,

                        (
                            "STATE: "
                            "CAMERA_CORRIDOR_CENTER"
                        ),

                        (
                            10,
                            145,
                        ),

                        cv2.FONT_HERSHEY_SIMPLEX,

                        0.55,

                        (
                            255,
                            255,
                            255,
                        ),

                        2,
                    )

                    cv2.imshow(
                        "Experimental Corridor Camera",
                        debug,
                    )


            # Confirmed panel loss ends camera authority permanently.
            elif state == ExperimentState.APPROACH_CORRIDOR:
                with sensor_lock:
                    scan = latest_scan
                    frame = (None if latest_forward_frame is None
                             else latest_forward_frame.copy())
                    cam_seq = camera_sequence

                if scan is None or scan.age_s > 0.30:
                    send_stop(master)
                    time.sleep(0.02)
                    continue

                front = panel_front_distance(scan)
                if front is not None and front <= args.entrance_commit_range:
                    send_stop(master)
                    descent = None
                    descent_started = now
                    state = ExperimentState.DESCEND_BEFORE_PRE_ENTRY
                    print(f"[APPROACH] front={front:.2f} m: camera OFF "
                          f"-> DESCEND_BEFORE_PRE_ENTRY ({args.pre_entry_descent:g} m)")
                    continue

                if frame is None:
                    send_stop(master)
                elif cam_seq != last_camera_sequence:
                    last_camera_sequence = cam_seq
                    result = banner_detector.detect(frame, relaxed_approach=True)
                    approach_detection_diag = (
                        f"detected={result['detected']} area={result['area']:.0f}px2 "
                        f"largest={result['largest_area']:.0f}px2 clipped={result['clipped']} "
                        f"reason={result['rejection_reason']}")
                    if result["detected"]:
                        banner_lost_frames = 0
                        vy = clamp(float(result["error_x"]) * args.camera_gain,
                                   -args.camera_max_speed, args.camera_max_speed)
                        vz = clamp(float(result["error_y"]) * args.camera_gain,
                                   -args.camera_max_speed, args.camera_max_speed)
                        send_camera_velocity(master, abs(args.approach_speed), vy, vz)
                    else:
                        banner_lost_frames += 1
                        send_stop(master)
                        if banner_lost_frames >= args.banner_loss_frames:
                            descent = None
                            descent_started = now
                            state = ExperimentState.DESCEND_BEFORE_PRE_ENTRY
                            print(f"[APPROACH] loss reason: {approach_detection_diag}; front={front}")
                            print("[APPROACH] panel lost: camera OFF, forward STOP "
                                  f"-> DESCEND_BEFORE_PRE_ENTRY ({args.pre_entry_descent:g} m)")
                    cv2.imshow("Experimental Corridor Camera", result["debug_frame"])

                if now - last_diag >= 0.5:
                    range_text = "unavailable" if front is None else f"{front:.2f}m"
                    print(f"[APPROACH] panel loss frames="
                          f"{banner_lost_frames}/{args.banner_loss_frames} "
                          f"front={range_text} scan_age={scan.age_s:.2f}s "
                          f"{approach_detection_diag}")
                    last_diag = now

            elif state == ExperimentState.DESCEND_BEFORE_PRE_ENTRY:
                # NativeMissionRunner is deliberately not constructed until
                # descent finishes: PRE_ENTRY timers cannot run during descent.
                if descent is None:
                    fresh = (telemetry.z_m is not None
                             and telemetry.vz_m_s is not None
                             and math.isfinite(telemetry.z_m)
                             and math.isfinite(telemetry.vz_m_s)
                             and 0 <= now - telemetry.position_time <= 0.50)
                    if not fresh:
                        send_stop(master)
                        if now - descent_started >= 2.0:
                            print("[DESCENT] ABORT: no fresh altitude for start height")
                            state = ExperimentState.ABORT
                        time.sleep(0.02)
                        continue
                    # NED Z grows downward; target is the configured distance
                    # below the handoff height.
                    target_height = -telemetry.z_m - args.pre_entry_descent
                    try:
                        descent = AltitudeController(target=target_height,
                            max_speed=0.50, tolerance=0.10, dwell=0.5, timeout=60.0,
                            telemetry_max_age=1.5, gain=1.0)
                    except ValueError:
                        send_stop(master)
                        print("[DESCENT] ABORT: target too low relative to EKF origin")
                        state = ExperimentState.ABORT
                        continue
                    descent.start(now)
                    print(f"[DESCENT] start={-telemetry.z_m:.2f} m, "
                          f"target={target_height:.2f} m above EKF origin")

                result = descent.update(now, telemetry.z_m, telemetry.vz_m_s,
                                        telemetry.position_time,
                                        mission_time=telemetry.position_boot_s)
                if result.error:
                    send_stop(master)
                    print(f"[DESCENT] ABORT: {result.error}")
                    state = ExperimentState.ABORT
                elif result.ready:
                    send_stop(master)
                    hover_stable_since = None
                    hover_started = now
                    hover_boot_start = telemetry.position_boot_s
                    state = ExperimentState.HOVER_BEFORE_PRE_ENTRY
                    print("[DESCENT] settled -> HOVER_BEFORE_PRE_ENTRY (2 simulation seconds)")
                else:
                    send_camera_velocity(master, 0.0, 0.0, result.vz_down)
                    if now - last_diag >= 0.5:
                        print(f"[DESCENT] height={-telemetry.z_m:.2f} m "
                              f"target={descent.target:.2f} m "
                              f"vz_down={result.vz_down:+.2f} m/s "
                              f"sim_elapsed={telemetry.position_boot_s - descent.mission_started:.1f}s "
                              f"pose_age={now - telemetry.position_time:.2f}s "
                              f"measured_vz={telemetry.vz_m_s:+.2f} m/s")
                        last_diag = now

            elif state == ExperimentState.HOVER_BEFORE_PRE_ENTRY:
                send_stop(master)
                age = now - telemetry.position_time
                values = (telemetry.z_m, telemetry.vx_m_s,
                          telemetry.vy_m_s, telemetry.vz_m_s)
                valid = all(v is not None and math.isfinite(v) for v in values)
                if (now - hover_started >= 180.0 or age > 3.5
                        or telemetry.position_boot_s < hover_boot_start):
                    print("[HOVER] ABORT: hover timeout, telemetry loss or clock reset")
                    state = ExperimentState.ABORT
                elif not valid or not 0 <= age <= 1.5:
                    hover_stable_since = None
                elif abs(-telemetry.z_m - descent.target) > 0.15:
                    # Reacquire the same target, never another relative descent.
                    descent.start(now)
                    state = ExperimentState.DESCEND_BEFORE_PRE_ENTRY
                    print("[HOVER] altitude drift: reacquiring existing target")
                elif max(abs(telemetry.vx_m_s), abs(telemetry.vy_m_s),
                         abs(telemetry.vz_m_s)) > 0.10:
                    hover_stable_since = None
                else:
                    if hover_stable_since is None:
                        hover_stable_since = telemetry.position_boot_s
                    if telemetry.position_boot_s - hover_stable_since >= 2.0:
                        corridor = create_corridor_runner(args.enter_distance)
                        previous_native_state = None
                        last_scan_sequence = -1
                        state = ExperimentState.LIDAR_CORRIDOR
                        print("[HOVER] 2 s stable -> PRE_ENTRY_GEOMETRY_LOCK (fresh timers)")

            # =================================================
            # STATE 4+
            # REAL NATIVE LIDAR FSM
            # =================================================

            elif (
                state
                ==
                ExperimentState.
                LIDAR_CORRIDOR
            ):

                if corridor is None:

                    raise RuntimeError(
                        "LIDAR_CORRIDOR "
                        "without mission runner"
                    )


                with sensor_lock:

                    scan = latest_scan
                    scan_world_pose = latest_lidar_world_pose

                    scan_seq = (
                        scan_sequence
                    )


                if scan is None or scan.age_s > 0.30:

                    send_stop(
                        master
                    )

                    continue


                # One native FSM iteration per LiDAR scan.
                if (
                    scan_seq
                    != last_scan_sequence
                ):

                    last_scan_sequence = (
                        scan_seq
                    )


                    pose = (
                        current_pose()
                    )

                    attitude = (
                        current_attitude()
                    )


                    output = (
                        corridor.step(

                            scan=scan,

                            attitude=attitude,

                            pose=pose,
                        )
                    )


                    if not preentry_snapshot_saved:
                        try:
                            g = corridor.pre_entry.last_geometry
                            fields = ("confidence", "strict_valid", "loose_valid",
                                      "front_clearance", "width", "left_inliers",
                                      "right_inliers", "left_span", "right_span",
                                      "left_rms", "right_rms", "sectors")
                            geometry = {key: getattr(g, key, None) for key in fields}
                            args.debug_output.mkdir(parents=True, exist_ok=True)
                            np.savez(args.debug_output / "preentry_scan.npz",
                                     angles_rad=scan.angles_rad, ranges_m=scan.ranges_m,
                                     range_min_m=scan.range_min_m, range_max_m=scan.range_max_m)
                            details = {"lidar_world_pose": scan_world_pose,
                                       "ekf_z": telemetry.z_m, "geometry": geometry}
                            details["local_position_ned"] = {
                                "x": telemetry.x_m, "y": telemetry.y_m,
                                "z": telemetry.z_m, "yaw_rad": telemetry.yaw_rad,
                            }
                            (args.debug_output / "preentry_geometry.json").write_text(
                                json.dumps(details, indent=2))
                            print("[PRE_ENTRY SNAPSHOT] saved to", args.debug_output, details)
                            preentry_snapshot_saved = True
                        except Exception as exc:
                            print("[PRE_ENTRY SNAPSHOT] capture failed:", exc)
                            preentry_snapshot_saved = True

                    native_state = (
                        corridor.
                        public_state()
                    )


                    # -----------------------------------------
                    # REAL NATIVE COMMAND
                    # -----------------------------------------

                    if (
                        output.action
                        ==
                        VehicleAction.LAND
                    ):

                        send_stop(
                            master
                        )

                        request_land(
                            master
                        )

                    else:

                        command = output.command
                        send_native_velocity(master, command)


                    # -----------------------------------------
                    # STATE CHANGE
                    # -----------------------------------------

                    if (
                        native_state
                        !=
                        previous_native_state
                    ):

                        print()

                        print(
                            "[NATIVE FSM] ->",
                            native_state.value,
                        )

                        previous_native_state = (
                            native_state
                        )


                    # -----------------------------------------
                    # DIAGNOSTICS
                    # -----------------------------------------

                    if (
                        now - last_diag
                        >= 0.50
                    ):

                        cmd = (output.command if output.action == VehicleAction.LAND
                               else command)
                        pos_age = now - telemetry.position_time
                        att_age = now - telemetry.attitude_time
                        entry_progress = (
                            f" | {output.reason}"
                            if (
                                native_state
                                == MissionState.ENTER_CORRIDOR
                                and output.reason
                            )
                            else ""
                        )
                        if native_state == MissionState.EXIT_DETECTION:
                            distance = corridor.exit.measured_travel_m
                            measured = "pending" if distance is None else f"{distance:.2f}m"
                            entry_progress = (
                                f" | exit_travel={measured}/"
                                f"{corridor.exit.commit_distance():.2f}m"
                                f" sim_elapsed={corridor.exit.elapsed_s():.2f}s"
                                f"/{corridor.exit.config.exit_hard_timeout_s:.1f}s"
                            )

                        print(

                            f"[{native_state.value:<25}] "

                            f"vx="
                            f"{cmd.vx_m_s:+.3f} "

                            f"vy="
                            f"{cmd.vy_m_s:+.3f} "

                            f"vz="
                            f"{cmd.vz_m_s:+.3f} "

                            f"yaw="
                            f"{math.degrees(cmd.yaw_rate_rad_s):+.1f}"
                            f"deg/s "

                            f"| status="
                            f"{output.status} "

                            f"| conf="
                            f"{output.confidence} "

                            f"| scan="
                            f"{scan.age_s:.3f}s "

                            f"| pose="
                            f"{'OK' if pose else 'NO'} "

                            f"| position_age={pos_age:.2f}s "

                            f"attitude_age={att_age:.2f}s "

                            f"position_messages={telemetry.position_messages}"
                            f"{entry_progress}"
                        )

                        last_diag = now


                    # -----------------------------------------
                    # ABORT
                    # -----------------------------------------

                    if (
                        native_state
                        ==
                        MissionState.
                        ABORT_CORRIDOR
                    ):

                        state = (
                            ExperimentState.
                            ABORT
                        )


                        print()

                        print(
                            "=========================================="
                        )

                        print(
                            " EXPERIMENT ABORTED"
                        )

                        print(
                            "=========================================="
                        )


                    # -----------------------------------------
                    # SUCCESS
                    # -----------------------------------------

                    elif (
                        native_state
                        ==
                        MissionState.
                        CORRIDOR_EXITED
                    ):

                        send_stop(
                            master
                        )
                        # Field starts beyond the corridor roof. This catches
                        # gross spawn/registration errors, not subtle EKF drift.
                        registered = coverage_entry_registered(telemetry, coverage_cfg, now)
                        if not registered:
                            print("[COVERAGE] ABORT: corridor exit outside registered field")
                            state = ExperimentState.ABORT
                        else:
                            field_advance_started = now
                            field_advance_boot_start = telemetry.position_boot_s
                            state = ExperimentState.ADVANCE_TO_FIELD


                        print()

                        print(
                            "=========================================="
                        )

                        print(
                            " EXIT_DETECTION COMPLETE"
                        )

                        print(
                            " CORRIDOR EXITED"
                        )

                        print(" ADVANCE_TO_FIELD" if registered else " COVERAGE ABORT")

                        print(
                            "=========================================="
                        )

            elif state == ExperimentState.ADVANCE_TO_FIELD:
                fresh = (telemetry.x_m is not None and telemetry.vx_m_s is not None
                         and math.isfinite(telemetry.x_m) and math.isfinite(telemetry.vx_m_s)
                         and 0 <= now - telemetry.position_time <= LIDAR_POSE_MAX_AGE_S)
                if (now - field_advance_started > 30.0
                        or telemetry.position_boot_s < field_advance_boot_start
                        or telemetry.position_boot_s - field_advance_boot_start > 12.0):
                    send_stop(master)
                    print("[COVERAGE] ABORT: field-entry progress timed out")
                    state = ExperimentState.ABORT
                elif not fresh:
                    send_stop(master)
                    if now - telemetry.position_time > 2.0:
                        print("[COVERAGE] ABORT: field-entry localization stale")
                        state = ExperimentState.ABORT
                elif telemetry.x_m < coverage_cfg.n_min + coverage_cfg.clearance + .4:
                    # Continue north beyond the roof and inside the coverage
                    # planner's clearance inset before climbing vertically.
                    send_camera_velocity(master, .15, 0.0, 0.0)
                elif abs(telemetry.vx_m_s) > .10:
                    send_stop(master)
                else:
                    send_stop(master)
                    relative_fresh = (telemetry.relative_alt_m is not None
                                      and math.isfinite(telemetry.relative_alt_m)
                                      and 0 <= now - telemetry.relative_alt_time <= 1.0)
                    if not relative_fresh or telemetry.z_m is None:
                        print("[COVERAGE] ABORT: HOME-relative altitude unavailable")
                        state = ExperimentState.ABORT
                    else:
                        # The EKF local-Z datum is not HOME relative in this
                        # world. Translate the measured datum for the climb;
                        # coverage projection itself uses relative_alt.
                        local_minus_home = -telemetry.z_m - telemetry.relative_alt_m
                        target_local = coverage_cfg.altitude + local_minus_home
                        coverage_climb = AltitudeController(
                            target=target_local,
                            max_speed=0.50, tolerance=0.15, dwell=1.0,
                            timeout=90.0, telemetry_max_age=1.5, gain=0.8,
                        )
                        coverage_climb.start(now)
                        state = ExperimentState.ASCEND_FOR_COVERAGE
                        print(f"[COVERAGE] field entry N={telemetry.x_m:.2f}; "
                              f"HOME/EKF offset={local_minus_home:.2f} m")

            elif state == ExperimentState.ASCEND_FOR_COVERAGE:
                result = coverage_climb.update(
                    now, telemetry.z_m, telemetry.vz_m_s,
                    telemetry.position_time, mission_time=telemetry.position_boot_s,
                )
                if result.error:
                    send_stop(master)
                    print(f"[COVERAGE] climb aborted: {result.error}")
                    state = ExperimentState.ABORT
                elif result.ready:
                    send_stop(master)
                    state = ExperimentState.COVERAGE
                    print("[COVERAGE] 10 m settled; handing sole control to coverage runtime")
                else:
                    send_camera_velocity(master, 0.0, 0.0, result.vz_down)

            elif state == ExperimentState.COVERAGE:
                # Reuse this MAVLink connection. The corridor loop does not
                # publish commands while the coverage runtime is running.
                coverage_started = True
                try:
                    coverage_exit = run_coverage([
                        "--config", str(args.coverage_config),
                        "--log", str(args.coverage_log),
                        "--max-wall-seconds", str(args.coverage_max_wall_seconds),
                        "--entry-wall-seconds", "30",
                        "--fly",
                    ], master=master)
                except (Exception, SystemExit) as exc:
                    # Setup failed before the coverage runtime's guarded loop.
                    # It has not sent motion commands, so the manager retains
                    # stop authority for this exceptional startup path.
                    coverage_started = False
                    print(f"[COVERAGE] startup failed: {exc}")
                    state = ExperimentState.ABORT
                else:
                    state = (ExperimentState.COMPLETE if coverage_exit == 0
                             else ExperimentState.ABORT)
                    print(f"[COVERAGE] {'COMPLETE' if coverage_exit == 0 else 'ABORT'}")


            # =================================================
            # COMPLETE
            # =================================================

            elif (
                state
                ==
                ExperimentState.COMPLETE
            ):
                break


            # =================================================
            # ABORT
            # =================================================

            elif (
                state
                ==
                ExperimentState.ABORT
            ):
                if not coverage_started:
                    send_stop(master)

                break


            # =================================================
            # GUI ESC
            # =================================================

            if (
                cv2.waitKey(1)
                & 0xFF
                == 27
            ):

                print(
                    "[EXPERIMENT] "
                    "ESC pressed"
                )

                break


            # =================================================
            # LOOP RATE
            # =================================================

            elapsed = (
                time.monotonic()
                -
                loop_started
            )

            time.sleep(

                max(
                    0.0,
                    0.02 - elapsed,
                )
            )


    except KeyboardInterrupt:

        print()

        print(
            "[EXPERIMENT] "
            "KeyboardInterrupt"
        )
        state = ExperimentState.ABORT


    finally:

        print()

        if not coverage_started:
            print("[EXPERIMENT] sending STOP")
            for _ in range(10):
                try:
                    send_stop(master)
                except Exception:
                    pass
                time.sleep(0.05)
        else:
            print("[EXPERIMENT] coverage owns the final hold; no second command")

        master.close()

        cv2.destroyAllWindows()

        print(
            "[EXPERIMENT] "
            "manager stopped"
        )

    return 0 if state == ExperimentState.COMPLETE else 1


if __name__ == "__main__":
    raise SystemExit(main())
