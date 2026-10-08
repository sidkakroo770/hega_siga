#!/usr/bin/env python3

from __future__ import annotations

import os

# Must be set before Gazebo protobuf imports.
os.environ.setdefault(
    "PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION",
    "python",
)

import argparse
import math
import sys
import threading
import time

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

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

from native.controllers.pre_entry import (
    PreEntryConfig,
    PreEntryController,
)


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

    LIDAR_CORRIDOR = (
        "LIDAR_CORRIDOR"
    )

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

        scan_sequence += 1


# ============================================================
# MAVLINK TELEMETRY CACHE
# ============================================================

@dataclass
class Telemetry:

    x_m: Optional[float] = None
    y_m: Optional[float] = None

    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None

    position_time: float = 0.0
    attitude_time: float = 0.0


telemetry = Telemetry()


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

            telemetry.position_time = now

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
        > 0.50
    ):
        return None

    if (
        now - telemetry.attitude_time
        > 0.50
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


# ============================================================
# MAIN
# ============================================================

def main() -> None:

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
            "centered and LiDAR searches for "
            "trustworthy corridor geometry."
        ),
    )

    parser.add_argument(
        "--handoff-scans",

        type=int,

        default=3,

        help=(
            "Consecutive trustworthy LiDAR "
            "corridor scans required before "
            "camera -> LiDAR handoff."
        ),
    )

    parser.add_argument(
        "--probe-timeout",

        type=float,

        default=60.0,

        help=(
            "Geometry-probe acquire timeout. "
            "Does not modify the actual corridor "
            "FSM configuration."
        ),
    )

    parser.add_argument(
        "--banner-loss-frames",

        type=int,

        default=5,
    )

    parser.add_argument(
        "--entrance-commit-range",

        type=float,

        default=1.20,

        help=(
            "Once LiDAR front clearance falls below this "
            "distance during APPROACH_CORRIDOR, the vehicle "
            "is considered committed to corridor entry. "
            "After that point banner loss triggers LiDAR handoff "
            "instead of returning to BANNER_SEARCH."
        ),
    )


    args = parser.parse_args()


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
        "APPROACH_CORRIDOR"
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

    print()

    print(
        "APPROACH speed:",
        f"{args.approach_speed:.2f} m/s",
    )

    print(
        "LiDAR handoff scans:",
        args.handoff_scans,
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


    # Request faster pose streams.
    request_message_interval(

        master,

        mavutil.mavlink.
        MAVLINK_MSG_ID_LOCAL_POSITION_NED,

        20.0,
    )

    request_message_interval(

        master,

        mavutil.mavlink.
        MAVLINK_MSG_ID_ATTITUDE,

        20.0,
    )


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


    # This is only the geometry watcher used during
    # APPROACH_CORRIDOR.
    #
    # It NEVER commands the aircraft.
    geometry_probe: Optional[
        PreEntryController
    ] = None


    # Actual corridor FSM.
    #
    # Created ONLY when LiDAR handoff happens.
    corridor: Optional[
        NativeMissionRunner
    ] = None


    centered_frames = 0

    banner_lost_frames = 0

    handoff_good_scans = 0

    # Once this becomes True during APPROACH_CORRIDOR,
    # visual navigation is not allowed to return to
    # BANNER_SEARCH. Banner disappearance then means
    # we have crossed / entered the corridor mouth.
    entrance_committed = False

    latest_probe_geometry = None


    last_camera_sequence = -1
    last_scan_sequence = -1
    last_probe_scan_sequence = -1

    last_diag = 0.0

    previous_native_state = None


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


                            # ---------------------------------
                            # Geometry probe uses EXACT SAME
                            # PRE_ENTRY geometry thresholds.
                            #
                            # Only acquisition timeout is made
                            # longer because it is being used
                            # as a proximity detector here.
                            # ---------------------------------

                            probe_config = (
                                PreEntryConfig(

                                    acquire_timeout_s=(
                                        args.probe_timeout
                                    )
                                )
                            )

                            geometry_probe = (
                                PreEntryController(
                                    config=(
                                        probe_config
                                    )
                                )
                            )

                            geometry_probe.enter()


                            handoff_good_scans = 0

                            banner_lost_frames = 0

                            entrance_committed = False

                            latest_probe_geometry = None

                            last_probe_scan_sequence = (
                                -1
                            )


                            state = (
                                ExperimentState.
                                APPROACH_CORRIDOR
                            )


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
                                " LiDAR now probes corridor geometry."
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


            # =================================================
            # STATE 3
            # APPROACH CORRIDOR
            #
            # CAMERA COMMANDS THE AIRCRAFT.
            # LIDAR ONLY OBSERVES.
            # =================================================

            elif (
                state
                ==
                ExperimentState.
                APPROACH_CORRIDOR
            ):

                if geometry_probe is None:

                    raise RuntimeError(
                        "APPROACH_CORRIDOR "
                        "without geometry probe"
                    )


                # ---------------------------------------------
                # FIRST: inspect new LiDAR scan.
                # ---------------------------------------------

                with sensor_lock:

                    scan = latest_scan

                    lidar_seq = (
                        scan_sequence
                    )


                if (
                    scan is not None
                    and
                    lidar_seq
                    != last_probe_scan_sequence
                ):

                    last_probe_scan_sequence = (
                        lidar_seq
                    )

                    attitude = (
                        current_attitude()
                    )


                    # IMPORTANT:
                    #
                    # During APPROACH_CORRIDOR we do NOT run the
                    # PRE_ENTRY FSM itself.
                    #
                    # Calling step() here previously allowed the
                    # probe to timeout into HOLD /
                    # LOW_CONFIDENCE_GEOMETRY before the drone
                    # physically reached the corridor.
                    #
                    # We only extract geometry passively.
                    g = (
                        geometry_probe.
                        extract_corridor_geometry(
                            scan
                        )
                    )

                    latest_probe_geometry = g


                    trustworthy = (

                        g.strict_valid

                        and

                        g.confidence
                        >=
                        geometry_probe.
                        config.
                        control_confidence_min

                        and

                        g.front_clearance
                        >=
                        geometry_probe.
                        config.
                        front_stop_m
                    )


                    # -----------------------------------------
                    # ENTRANCE COMMIT LATCH
                    #
                    # Once the drone is this close to the entrance
                    # object / banner, visual search is no longer
                    # allowed to restart.
                    # -----------------------------------------

                    if (
                        not entrance_committed
                        and
                        g.front_clearance
                        <=
                        args.entrance_commit_range
                    ):

                        entrance_committed = True

                        print()

                        print(
                            "=========================================="
                        )

                        print(
                            " ENTRANCE COMMITTED"
                        )

                        print(
                            f" front clearance="
                            f"{g.front_clearance:.2f} m"
                        )

                        print(
                            " Banner loss will now trigger"
                        )

                        print(
                            " CAMERA -> LIDAR handoff"
                        )

                        print(
                            "=========================================="
                        )

                        print()


                    if trustworthy:

                        handoff_good_scans += 1

                    else:

                        handoff_good_scans = 0


                    if (
                        now - last_diag
                        >= 0.40
                    ):

                        if g is None:

                            print(
                                "[APPROACH] "
                                f"vx="
                                f"{args.approach_speed:+.2f} "
                                "| LiDAR geometry unavailable "
                                f"| handoff="
                                f"{handoff_good_scans}/"
                                f"{args.handoff_scans}"
                            )

                        else:

                            print(

                                "[APPROACH] "

                                f"vx="
                                f"{args.approach_speed:+.2f} "

                                f"| committed="
                                f"{entrance_committed} "

                                f"| conf="
                                f"{g.confidence:.3f} "

                                f"| strict="
                                f"{g.strict_valid} "

                                f"| width="
                                f"{g.width:.2f}m "

                                f"| front="
                                f"{g.front_clearance:.2f}m "

                                f"| handoff="
                                f"{handoff_good_scans}/"
                                f"{args.handoff_scans}"
                            )

                        last_diag = now


                    # -----------------------------------------
                    # LIDAR PROVES WE ARE CLOSE ENOUGH.
                    # -----------------------------------------

                    if (
                        handoff_good_scans
                        >=
                        args.handoff_scans
                    ):

                        send_stop(
                            master
                        )


                        print()

                        print(
                            "=========================================="
                        )

                        print(
                            " TRUSTWORTHY CORRIDOR GEOMETRY ACQUIRED"
                        )

                        print(
                            " CAMERA AUTHORITY ENDS"
                        )

                        print(
                            " -> PRE_ENTRY_GEOMETRY_LOCK"
                        )

                        print(
                            "=========================================="
                        )

                        print()


                        # -------------------------------------
                        # NOW create the REAL native FSM.
                        #
                        # Fresh controller,
                        # fresh timers,
                        # no probe failure state carried over.
                        # -------------------------------------

                        corridor_config = (
                            MissionRunnerConfig(

                                enter_corridor_distance_m=(
                                    args.enter_distance
                                )
                            )
                        )

                        corridor = (
                            NativeMissionRunner(

                                config=(
                                    corridor_config
                                )
                            )
                        )


                        previous_native_state = (
                            None
                        )

                        last_scan_sequence = (
                            -1
                        )

                        state = (
                            ExperimentState.
                            LIDAR_CORRIDOR
                        )

                        continue


                # ---------------------------------------------
                # SECOND: camera keeps banner aligned while
                # moving slowly forward.
                # ---------------------------------------------

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


                    if result[
                        "detected"
                    ]:

                        banner_lost_frames = 0


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


                        vy = clamp(

                            error_x
                            *
                            args.camera_gain,

                            -args.camera_max_speed,

                            args.camera_max_speed,
                        )


                        vz = clamp(

                            error_y
                            *
                            args.camera_gain,

                            -args.camera_max_speed,

                            args.camera_max_speed,
                        )


                        # THIS is the important change:
                        #
                        # camera alignment +
                        # deliberate slow forward approach.

                        send_camera_velocity(

                            master,

                            vx=(
                                abs(
                                    args.approach_speed
                                )
                            ),

                            vy=vy,

                            vz=vz,
                        )


                    else:

                        banner_lost_frames += 1

                        send_stop(
                            master
                        )


                        if (
                            banner_lost_frames
                            >=
                            args.banner_loss_frames
                        ):

                            # =================================
                            # CASE 1:
                            # We are already close enough to the
                            # corridor entrance.
                            #
                            # Banner disappearance here is EXPECTED.
                            # It means visual navigation has finished.
                            # =================================

                            if entrance_committed:

                                print()

                                print(
                                    "=========================================="
                                )

                                print(
                                    " BANNER LOST AFTER ENTRANCE COMMIT"
                                )

                                print(
                                    " Visual entrance phase complete"
                                )

                                print(
                                    " CAMERA AUTHORITY ENDS"
                                )

                                print(
                                    " -> PRE_ENTRY_GEOMETRY_LOCK"
                                )

                                print(
                                    "=========================================="
                                )

                                print()


                                corridor_config = (
                                    MissionRunnerConfig(

                                        enter_corridor_distance_m=(
                                            args.enter_distance
                                        )
                                    )
                                )

                                corridor = (
                                    NativeMissionRunner(

                                        config=(
                                            corridor_config
                                        )
                                    )
                                )

                                previous_native_state = (
                                    None
                                )

                                last_scan_sequence = (
                                    -1
                                )

                                state = (
                                    ExperimentState.
                                    LIDAR_CORRIDOR
                                )

                                geometry_probe = None

                                continue


                            # =================================
                            # CASE 2:
                            # Banner disappeared while still far
                            # away.
                            #
                            # That is a genuine visual tracking loss.
                            # =================================

                            else:

                                print()

                                print(
                                    "[APPROACH] "
                                    "banner lost before "
                                    "entrance commitment"
                                )

                                print(
                                    "[APPROACH] "
                                    "STOP -> BANNER_SEARCH"
                                )

                                geometry_probe = None

                                centered_frames = 0

                                handoff_good_scans = 0

                                state = (
                                    ExperimentState.
                                    BANNER_SEARCH
                                )


                    debug = (
                        result[
                            "debug_frame"
                        ]
                    )

                    cv2.putText(

                        debug,

                        (
                            "STATE: "
                            "APPROACH_CORRIDOR"
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

                    scan_seq = (
                        scan_sequence
                    )


                if scan is None:

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

                        send_native_velocity(

                            master,

                            output.command,
                        )


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

                        cmd = (
                            output.command
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
                            f"{'OK' if pose else 'NO'}"
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

                        state = (
                            ExperimentState.
                            COMPLETE
                        )


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

                        print(
                            " EXPERIMENT COMPLETE"
                        )

                        print(
                            "=========================================="
                        )


            # =================================================
            # COMPLETE
            # =================================================

            elif (
                state
                ==
                ExperimentState.COMPLETE
            ):

                send_stop(
                    master
                )

                break


            # =================================================
            # ABORT
            # =================================================

            elif (
                state
                ==
                ExperimentState.ABORT
            ):

                send_stop(
                    master
                )

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


    finally:

        print()

        print(
            "[EXPERIMENT] "
            "sending STOP"
        )

        for _ in range(10):

            try:

                send_stop(
                    master
                )

            except Exception:
                pass

            time.sleep(
                0.05
            )

        cv2.destroyAllWindows()

        print(
            "[EXPERIMENT] "
            "manager stopped"
        )


if __name__ == "__main__":
    main()
