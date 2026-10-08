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

from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Optional

from corridor_altitude import AltitudeController
from corridor_handoff import panel_front_distance

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

    ACQUIRE_CORRIDOR_ALTITUDE = "ACQUIRE_CORRIDOR_ALTITUDE"

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

    z_m: Optional[float] = None
    vz_m_s: Optional[float] = None
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

            telemetry.z_m = float(msg.z)
            telemetry.vz_m_s = float(msg.vz)
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
        "--banner-loss-frames",

        type=int,

        default=5,
    )

    parser.add_argument(
        "--entrance-commit-range",

        type=float,

        default=1.0,

        help=(
            "Once LiDAR front clearance falls below this "
            "distance during APPROACH_CORRIDOR, the vehicle "
            "stops camera control and descends, then starts "
            "PRE_ENTRY_GEOMETRY_LOCK."
        ),
    )


    parser.add_argument("--corridor-altitude", type=float, default=1.3,
                        help="Vehicle height above EKF origin (=-LOCAL_POSITION_NED.z), not AGL")
    parser.add_argument("--altitude-max-speed", type=float, default=0.30)
    parser.add_argument("--altitude-tolerance", type=float, default=0.12)
    parser.add_argument("--altitude-dwell", type=float, default=1.0)
    parser.add_argument("--altitude-timeout", type=float, default=20.0)
    args = parser.parse_args()
    if not math.isfinite(args.entrance_commit_range) or args.entrance_commit_range <= 0:
        parser.error("--entrance-commit-range must be finite and positive")
    try:
        altitude = AltitudeController(args.corridor_altitude,
                                     args.altitude_max_speed,
                                     args.altitude_tolerance,
                                     args.altitude_dwell,
                                     args.altitude_timeout)
    except ValueError as exc:
        parser.error(str(exc))
    print(f"[ALTITUDE] target={args.corridor_altitude:.2f} m above EKF origin")


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
        "APPROACH_CORRIDOR -> ACQUIRE_CORRIDOR_ALTITUDE"
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


    # Actual corridor FSM.
    #
    # Created ONLY when LiDAR handoff happens.
    corridor: Optional[
        NativeMissionRunner
    ] = None


    centered_frames = 0

    banner_lost_frames = 0

    last_camera_sequence = -1
    last_scan_sequence = -1

    last_diag = 0.0

    previous_native_state = None
    altitude_active = False


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


            # One vertical authority from acquisition through corridor exit.
            # Pause the native runner on missing altitude or while reacquiring.
            altitude_vz_down = 0.0
            if altitude_active and state not in (
                ExperimentState.COMPLETE, ExperimentState.ABORT
            ):
                altitude_result = altitude.update(
                    now, telemetry.z_m, telemetry.vz_m_s, telemetry.position_time
                )
                altitude_vz_down = altitude_result.vz_down
                if altitude_result.error:
                    print(f"[ALTITUDE] ABORT: {altitude_result.error}")
                    send_stop(master)
                    state = ExperimentState.ABORT
                elif not altitude_result.ready:
                    send_camera_velocity(master, 0.0, 0.0, altitude_vz_down)
                    if now - last_diag >= 0.5:
                        print(f"[ALTITUDE] acquiring target={altitude.target:.2f} "
                              f"z_NED={telemetry.z_m} vz_down={altitude_vz_down:+.2f}")
                        last_diag = now
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
                    time.sleep(0.02)
                    continue
                elif state == ExperimentState.ACQUIRE_CORRIDOR_ALTITUDE:
                    print("[ALTITUDE] settled -> PRE_ENTRY_GEOMETRY_LOCK")
                    corridor = NativeMissionRunner(config=MissionRunnerConfig(
                        enter_corridor_distance_m=args.enter_distance))
                    previous_native_state = None
                    last_scan_sequence = -1
                    state = ExperimentState.LIDAR_CORRIDOR

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


                            banner_lost_frames = 0

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
                                " LiDAR stops approach at the panel range threshold."
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


            # Range handoff runs before camera processing, even on banner loss.
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
                    altitude.start(now)
                    altitude_active = True
                    state = ExperimentState.ACQUIRE_CORRIDOR_ALTITUDE
                    print(f"[APPROACH] panel range={front:.2f} m: "
                          "camera lock OFF, forward STOP -> ACQUIRE_CORRIDOR_ALTITUDE")
                    continue

                if frame is None:
                    send_stop(master)
                elif cam_seq != last_camera_sequence:
                    last_camera_sequence = cam_seq
                    result = banner_detector.detect(frame)
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
                            # Do not resume lateral searching near an entrance.
                            # Wait for a new detection or the range trigger.
                            print("[APPROACH] banner lost: holding for camera/range")
                    cv2.imshow("Experimental Corridor Camera", result["debug_frame"])

                if now - last_diag >= 0.5:
                    print(f"[APPROACH] front={front} m, "
                          f"stop range={args.entrance_commit_range:.2f} m")
                    last_diag = now

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
                        if native_state not in (
                            MissionState.ABORT_CORRIDOR,
                            MissionState.CORRIDOR_EXITED,
                            MissionState.HOVER_AND_REASSESS,
                        ):
                            command = replace(command, vz_m_s=-altitude_vz_down)
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
