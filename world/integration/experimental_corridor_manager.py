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
from contextlib import contextmanager
import math
import sys
import threading
import time

from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Optional


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
from native.common.banner_guard import BannerGuard, BannerGuardConfig
from native.common.validity import attitude_valid, pose_valid, scan_valid, sector_clearance
from command_service import CommandService
from camera_frame import decode_gazebo_bgr, gazebo_source_stamp_ns
from entrance_readiness import EntranceReadiness, StagingEnvelope
from pose_integrity import PoseIntegrity
from preview_service import PreviewService
from snapshot_service import SnapshotService
from health_metrics import HealthMetrics
from async_log import AsyncLogStream
from source_clock import SourceClockGate
from startup_control import StartupConfig, StartupController, StartupFeedback

from coverage_mission.config import Config as CoverageConfig
from coverage_mission.field_frame import FieldFrame
from coverage_mission.runtime import main as run_coverage, Sensors as DownwardSensors
from coverage_mission.qr import decoder_self_check
from coverage_mission.return_mission import ReturnConfig
from return_approach import ReturnApproach




# ============================================================
# EXPERIMENT FSM
# ============================================================

class ExperimentState(str, Enum):

    STARTUP = "STARTUP"
    INITIAL_QR = "INITIAL_QR"

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
    RETURN_EGRESS = "RETURN_EGRESS"
    LANDING = "LANDING"

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
health_metrics = HealthMetrics()
camera_clock_gate = SourceClockGate("forward camera")
lidar_clock_gate = SourceClockGate("LiDAR")

latest_forward_frame: Optional[
    np.ndarray
] = None

latest_scan: Optional[
    NativeScan
] = None

latest_lidar_world_pose = None

camera_sequence = 0
camera_receipt_time = 0.0
camera_source_stamp_ns = None
forward_camera_active = True
forward_camera_generation = 0
scan_sequence = 0
forward_processing_width = 640  # Gazebo regression mode; not IMX296 native resolution.
camera_decode_errors = 0
last_camera_error_report = 0.0


# ============================================================
# CAMERA CALLBACK
# ============================================================

def on_forward_image(
    msg: Image,
) -> None:

    global latest_forward_frame
    global camera_sequence
    global camera_receipt_time
    global camera_source_stamp_ns
    global camera_decode_errors, last_camera_error_report

    with sensor_lock:
        if not forward_camera_active: return
        generation=forward_camera_generation

    received = time.monotonic()

    try:

        image = decode_gazebo_bgr(msg, forward_processing_width)

    except Exception as exc:
        camera_decode_errors += 1
        if received - last_camera_error_report >= 1.0:
            print(f"[CAMERA] decode errors={camera_decode_errors}: {exc}")
            last_camera_error_report = received

        return

    health_metrics.observe("camera_decode", time.monotonic() - received)

    with sensor_lock:
        if not forward_camera_active or generation!=forward_camera_generation: return
        source_stamp = gazebo_source_stamp_ns(msg)
        if not camera_clock_gate.accept(source_stamp):
            return

        latest_forward_frame = image

        camera_sequence += 1
        camera_receipt_time = received
        camera_source_stamp_ns = source_stamp


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
        no_return_is_clear=True,  # Gazebo ray sensor's documented adapter convention
        timestamp=time.monotonic(),
        range_min_m=float(
            msg.range_min
        ),
        range_max_m=float(
            msg.range_max
        ),
        source_timestamp=gazebo_source_stamp_ns(msg),
        source_clock="gazebo_sim_ns",
    )

    with sensor_lock:
        if not lidar_clock_gate.accept(scan.source_timestamp):
            return

        latest_scan = scan
        pose = msg.world_pose
        latest_lidar_world_pose = {
            "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
            "qx": pose.orientation.x, "qy": pose.orientation.y,
            "qz": pose.orientation.z, "qw": pose.orientation.w,
        }

        scan_sequence += 1
    health_metrics.observe("lidar_adapter", time.monotonic() - scan.timestamp)


def detect_banner(detector, frame, **options):
    started = time.monotonic()
    result = detector.detect(frame, **options)
    health_metrics.observe("banner_detect", time.monotonic() - started)
    return result


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
    origin_lat_deg: Optional[float] = None
    origin_lon_deg: Optional[float] = None
    origin_alt_mm: Optional[int] = None
    heartbeat_time: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    authority_revoked: bool = False
    landing_expected: bool = False
    authority_started: bool = False
    landed_state: Optional[int] = None
    landed_time: float = 0.0
    ekf_flags: int = 0
    ekf_time: float = 0.0
    prearm_ok: bool = False
    prearm_time: float = 0.0
    rejected_command: Optional[int] = None
    rejection_time: float = 0.0
    last_land_request: float = 0.0
    ack_command: Optional[int] = None
    ack_result: Optional[int] = None
    ack_time: float = 0.0
    pose_fault: Optional[str] = None


telemetry = Telemetry()
pose_integrity = PoseIntegrity()


def coverage_entry_registered(pose_telemetry, cfg, now):
    """Guard the configured field frame before advancing clear of the roof."""
    n, e = pose_telemetry.x_m, pose_telemetry.y_m
    if n is not None and e is not None and math.isfinite(n) and math.isfinite(e):
        n,e = FieldFrame(cfg).point((n,e))
    return bool(
        n is not None and e is not None
        and math.isfinite(n) and math.isfinite(e)
        and cfg.n_min + cfg.body_radius <= n <= cfg.n_max - cfg.clearance
        and cfg.e_min + cfg.clearance <= e <= cfg.e_max - cfg.clearance
        and 0 <= now - pose_telemetry.position_time <= LIDAR_POSE_MAX_AGE_S
    )


def north_velocity_in_body(speed, yaw_rad):
    """NED north-only velocity -> body forward/right (not FLU left)."""
    if not math.isfinite(speed) or not math.isfinite(yaw_rad):
        raise ValueError("Nonfinite field-advance velocity or heading")
    return speed * math.cos(yaw_rad), -speed * math.sin(yaw_rad)


def heading_velocity_in_body(speed, desired_yaw, actual_yaw):
    if not all(map(math.isfinite,(speed,desired_yaw,actual_yaw))):
        raise ValueError('Nonfinite field-advance command')
    error=desired_yaw-actual_yaw
    return speed*math.cos(error),speed*math.sin(error)


def field_advance_health(pose_telemetry, cfg, now):
    values = (pose_telemetry.z_m, pose_telemetry.vx_m_s,
              pose_telemetry.vy_m_s, pose_telemetry.yaw_rad)
    return (coverage_entry_registered(pose_telemetry, cfg, now)
            and all(v is not None and math.isfinite(v) for v in values)
            and 0 <= now - pose_telemetry.attitude_time <= .5)

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

    # Bound callback work so a traffic burst cannot starve command expiry.
    for _ in range(256):

        msg = master.recv_match(
            blocking=False,
        )

        if msg is None:
            break

        now = time.monotonic()

        kind = msg.get_type()
        if (msg.get_srcSystem() != master.target_system
                or msg.get_srcComponent() != master.target_component):
            continue

        if kind == "HEARTBEAT":
            telemetry.heartbeat_time = now
            telemetry.mode = mavutil.mode_string_v10(msg)
            telemetry.armed = bool(msg.base_mode &
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            expected_land=telemetry.landing_expected and telemetry.mode == "LAND"
            if telemetry.authority_started and not expected_land and (
                    telemetry.mode != "GUIDED" or not telemetry.armed):
                telemetry.authority_revoked = True
        elif kind == "EXTENDED_SYS_STATE":
            telemetry.landed_state = int(msg.landed_state)
            telemetry.landed_time = now
        elif kind == "EKF_STATUS_REPORT":
            telemetry.ekf_flags = int(msg.flags)
            telemetry.ekf_time = now
        elif kind == "SYS_STATUS":
            prearm_bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
            telemetry.prearm_ok = bool(
                int(msg.onboard_control_sensors_present) & prearm_bit
                and int(msg.onboard_control_sensors_health) & prearm_bit)
            telemetry.prearm_time = now
        elif kind == "COMMAND_ACK":
            telemetry.ack_command = int(msg.command)
            telemetry.ack_result = int(msg.result)
            telemetry.ack_time = now
            if int(msg.result) not in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                telemetry.rejected_command = int(msg.command)
                telemetry.rejection_time = now

        elif (
            kind
            == "LOCAL_POSITION_NED"
        ):

            boot_s = float(msg.time_boot_ms) / 1000.0
            xyz = (float(msg.x), float(msg.y), float(msg.z))
            velocity = (float(msg.vx), float(msg.vy), float(msg.vz))
            if not pose_integrity.observe(boot_s, xyz, velocity):
                if pose_integrity.failure is not None:
                    telemetry.pose_fault = pose_integrity.failure
                    telemetry.position_time = 0.0
                continue

            telemetry.x_m = float(
                msg.x
            )

            telemetry.y_m = float(
                msg.y
            )

            telemetry.position_boot_s = boot_s
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
        elif kind == 'GPS_GLOBAL_ORIGIN':
            new_origin=(int(msg.latitude),int(msg.longitude),int(msg.altitude))
            old_origin=(telemetry.origin_lat_deg,telemetry.origin_lon_deg,
                        telemetry.origin_alt_mm)
            if old_origin[0] is not None and new_origin != (
                    round(old_origin[0]*1e7),round(old_origin[1]*1e7),old_origin[2]):
                telemetry.pose_fault='FC local origin changed during mission'
            telemetry.origin_lat_deg=new_origin[0]*1e-7
            telemetry.origin_lon_deg=new_origin[1]*1e-7
            telemetry.origin_alt_mm=new_origin[2]


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


def refresh_manager_after_qr(master, timeout=3.0):
    """Bounded read-owner handback. Old receipt timestamps are never refreshed.

    QR runtime drained this same connection while the manager was paused. Hold
    until genuinely new heartbeat, estimator and pose packets are received.
    """
    started=time.monotonic()
    request_pose_telemetry(master)
    while time.monotonic()-started<timeout:
        drain_mavlink(master)
        now=time.monotonic()
        if telemetry.authority_revoked or telemetry.pose_fault:
            return False
        send_stop(master)
        if (telemetry.mode=='GUIDED' and telemetry.armed
                and telemetry.heartbeat_time>=started and telemetry.ekf_time>=started
                and (telemetry.ekf_flags&55)==55
                and now-telemetry.position_time<=LIDAR_POSE_MAX_AGE_S
                and now-telemetry.relative_alt_time<=.5
                and attitude_valid(current_attitude())):
            return True
        time.sleep(.02)
    return False


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

active_commands: Optional[CommandService] = None
manager_command_token = None
mav_tx_lock = threading.RLock()


@contextmanager
def mav_tx_guard(timeout_s=0.2):
    if not mav_tx_lock.acquire(timeout=timeout_s):
        raise RuntimeError("MAVLink transmit lock busy beyond bounded wait")
    try:
        yield
    finally:
        mav_tx_lock.release()


def bounded_mavlink_call(send, *args):
    with mav_tx_guard():
        return send(*args)


def transmit_body_velocity(master, vx, vy, vz, yaw_rate):
    """Only the command worker calls this while it owns navigation output."""
    with mav_tx_guard():
        master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
            0., 0., 0., vx, vy, vz, 0., 0., 0., 0., yaw_rate)


def transmit_mission_velocity(master, vx, vy, vz, yaw_rate, frame='body'):
    if frame == 'body':
        transmit_body_velocity(master, vx, vy, vz, yaw_rate)
    elif frame == 'local':
        with mav_tx_guard():
            master.mav.set_position_target_local_ned_send(
                int(time.monotonic()*1000)&0xffffffff,
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 2503,
                0,0,0,vx,vy,vz,0,0,0,yaw_rate,0)
    else:
        raise ValueError('Unsupported command frame')


def stop_command_service():
    global active_commands
    service = active_commands
    if service is not None:
        service.stop()
        active_commands = None


def send_camera_velocity(
    master,
    vx: float,
    vy: float,
    vz: float,
    source_timestamp=None,
) -> None:

    """
    Same velocity packet style used by shhhh.

    MAV_FRAME_BODY_OFFSET_NED:

        +X = forward
        +Y = right
        +Z = down
    """

    if active_commands is not None:
        active_commands.publish(vx, vy, vz, source_timestamp=source_timestamp, token=manager_command_token)
        return

    type_mask = int(
        0b0000111111000111
    )

    bounded_mavlink_call(master.mav.send,

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
    source_timestamp=None,
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

    if active_commands is not None:
        active_commands.publish(command.vx_m_s, -command.vy_m_s,
                                -command.vz_m_s, -command.yaw_rate_rad_s,
                                source_timestamp=source_timestamp,token=manager_command_token)
        return

    type_mask = 1479

    bounded_mavlink_call(master.mav.set_position_target_local_ned_send,

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

    stop_command_service()

    print(
        "[SAFETY] LAND requested"
    )

    with mav_tx_guard():
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0,
        )
    telemetry.last_land_request = time.monotonic()


def set_forward_camera_active(node, active):
    """Suspend acquisition/decoding between outbound exit and orange approach.

    Generation checking discards a callback already decoding when suspended.
    The source-clock gate remains intact across this intentional acquisition gap.
    """
    global forward_camera_active, forward_camera_generation
    global latest_forward_frame,camera_receipt_time,camera_source_stamp_ns
    with sensor_lock:
        if forward_camera_active==active: return
        forward_camera_active=False
        forward_camera_generation+=1
        latest_forward_frame=None; camera_receipt_time=0.; camera_source_stamp_ns=None
    if active:
        if not node.subscribe(Image,'/iris/camera_forward/image_raw',on_forward_image):
            raise RuntimeError('Return forward camera subscription failed')
        with sensor_lock: forward_camera_active=True
    else:
        node.unsubscribe('/iris/camera_forward/image_raw')
    print(f'[CAMERA] Forward acquisition {"resumed for orange approach" if active else "suspended after outbound exit"}')


def return_sensor_snapshot():
    """Immutable/reference snapshot; acquisition already owns the camera arrays."""
    with sensor_lock:
        return (latest_forward_frame,camera_sequence,camera_receipt_time,
                camera_source_stamp_ns,latest_scan,scan_sequence,
                camera_clock_gate.failure or lidar_clock_gate.failure)


def landing_confirmed_feedback(feedback,now,started,profile):
    """Fresh FC touchdown evidence AND the surveyed exterior landing region."""
    required=(feedback.x_m,feedback.y_m,feedback.relative_alt_m,
              feedback.vx_m_s,feedback.vy_m_s,feedback.vz_m_s)
    if not all(v is not None and math.isfinite(v) for v in required): return False
    delta=np.array([feedback.x_m,feedback.y_m])-profile.entrance
    along=float(delta@profile.axis)
    across=float(delta@np.array([-profile.axis[1],profile.axis[0]]))
    return (0<=now-feedback.heartbeat_time<=2.5 and 0<=now-feedback.landed_time<=1.5
        and 0<=now-feedback.relative_alt_time<=1. and 0<=now-feedback.position_time<=1.
        and feedback.heartbeat_time>=started and feedback.landed_time>=started
        and feedback.mode=='LAND' and not feedback.armed
        and feedback.landed_state==mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        and abs(feedback.relative_alt_m)<.4
        and max(abs(feedback.vx_m_s),abs(feedback.vy_m_s),abs(feedback.vz_m_s))<.15
        and profile.far_mouth_distance+profile.landing_clearance-.3<=along<=profile.far_mouth_distance+profile.landing_clearance+.8
        and abs(across)<=profile.entry_half_width)


def confirm_terminal_land(master, timeout_s=3.0):
    """Require fresh LAND mode; an accepted ACK alone is not a mode change."""
    if telemetry.mode == "LAND" and time.monotonic() - telemetry.heartbeat_time <= 2.5:
        return True
    if telemetry.last_land_request <= 0:
        request_land(master)
    deadline = time.monotonic() + timeout_s
    retries = 0
    while time.monotonic() < deadline:
        drain_mavlink(master)
        now = time.monotonic()
        if (telemetry.mode == "LAND"
                and 0 <= now - telemetry.heartbeat_time <= 2.5):
            return True
        if (telemetry.ack_command == mavutil.mavlink.MAV_CMD_NAV_LAND
                and telemetry.ack_time >= telemetry.last_land_request
                and telemetry.ack_result not in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS)):
            return False
        if telemetry.mode not in ("GUIDED", "LAND") or (
                telemetry.authority_revoked and telemetry.mode != "LAND"):
            return False  # Pilot owns the aircraft or command changed mode.
        if now - telemetry.last_land_request >= 1.5 and retries < 1:
            request_land(master)
            retries += 1
        time.sleep(.05)
    return False


def request_startup_action(master, action, takeoff_altitude_m):
    """One normal ArduCopter action, issued only by StartupController."""
    if action == "MODE":
        modes = master.mode_mapping()
        if not modes or "GUIDED" not in modes:
            raise RuntimeError("GUIDED mode unavailable from flight controller")
        command = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        params = (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                  int(modes["GUIDED"]), 0, 0, 0, 0, 0)
    elif action == "ARM":
        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        params = (1, 0, 0, 0, 0, 0, 0)
    elif action == "TAKEOFF":
        command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        params = (0, 0, 0, 0, 0, 0, takeoff_altitude_m)
    else:
        raise ValueError(f"Unsupported startup action {action}")
    with mav_tx_guard():
        master.mav.command_long_send(
            master.target_system, master.target_component, command, 0, *params)


def startup_feedback(started):
    with sensor_lock:
        cam_stamp = camera_receipt_time
        scan_stamp = latest_scan.timestamp if scan_valid(latest_scan) else 0.0
    position = (telemetry.x_m, telemetry.y_m, telemetry.z_m)
    velocity = (telemetry.vx_m_s, telemetry.vy_m_s, telemetry.vz_m_s)
    return StartupFeedback(
        heartbeat_time=telemetry.heartbeat_time, mode=telemetry.mode,
        armed=telemetry.armed, landed_state=telemetry.landed_state,
        landed_time=telemetry.landed_time, ekf_flags=telemetry.ekf_flags,
        ekf_time=telemetry.ekf_time, position_time=telemetry.position_time,
        attitude_time=telemetry.attitude_time,
        relative_alt_time=telemetry.relative_alt_time,
        relative_alt_m=telemetry.relative_alt_m,
        position_m=position if all(v is not None for v in position) else None,
        velocity_m_s=velocity if all(v is not None for v in velocity) else None,
        camera_time=cam_stamp, lidar_time=scan_stamp,
        rejected_command=(telemetry.rejected_command if
                          telemetry.rejection_time >= started else None),
        prearm_ok=telemetry.prearm_ok, prearm_time=telemetry.prearm_time,
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

    with mav_tx_guard():
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0,
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
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 1.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2.0)
    stream_hz = max(1, int(round(hz)))
    with mav_tx_guard():
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


def create_corridor_runner(enter_distance, vehicle_width_m=0.65,
                           passage_side_margin_m=0.25,
                           max_cross_track_m=0.80,
                           max_yaw_error_deg=40.0):
    """Longer stage deadlines for the slow local Gazebo simulation."""
    runner = NativeMissionRunner(config=MissionRunnerConfig(
        enter_corridor_distance_m=enter_distance,
        vehicle_width_m=vehicle_width_m,
        passage_side_margin_m=passage_side_margin_m,
        enter_max_cross_track_m=max_cross_track_m,
        enter_max_yaw_error_deg=max_yaw_error_deg,
        enter_corridor_max_pose_age_s=LIDAR_POSE_MAX_AGE_S,
        enter_corridor_pose_timeout_s=6.0,
        pre_entry_hold_timeout_s=32.0,
        # The routed full simulation can make less than the commanded 0.20
        # m/s while retaining valid position feedback.  Keep the measured
        # 0.75 m entry requirement and allow sufficient time to reach it.
        enter_corridor_timeout_s=60.0,
        reassess_hard_timeout_s=48.0,
    ))
    runner.exit.config.max_cross_track_m = max_cross_track_m
    runner.exit.config.max_yaw_error_deg = max_yaw_error_deg
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
    global active_commands, forward_processing_width, manager_command_token

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
    parser.add_argument("--start-mission", action="store_true",
                        help="Explicitly authorize normal GUIDED arm and autonomous takeoff")
    parser.add_argument("--takeoff-altitude", type=float, default=5.0,
                        help="Gazebo profile HOME-relative takeoff height, metres")
    parser.add_argument('--coverage-only', action='store_true',
                        help='Explicit regression mode: skip initial and field QR tasks')
    parser.add_argument('--qr-only', action='store_true',
                        help='Regression endpoint: stop after matched target 5 m hold')
    parser.add_argument('--test-return-only',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--test-field-qr-only',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--test-field-qr-approach',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--return-config',type=Path,
                        default=MISSION_ROOT/'config/return_mission.json')


    # --------------------------------------------------------
    # Corridor entry
    # --------------------------------------------------------

    parser.add_argument(
        "--enter-distance",

        type=float,

        default=0.75,
    )
    parser.add_argument("--vehicle-width", type=float, default=0.65,
                        help="Provisional maximum horizontal aircraft width, metres")
    parser.add_argument("--passage-side-margin", type=float, default=0.25,
                        help="Provisional clearance required on each side, metres")
    parser.add_argument("--max-path-cross-track", type=float, default=0.80,
                        help="Provisional straight entry/exit pose drift gate, metres")
    parser.add_argument("--max-path-yaw-error", type=float, default=40.0,
                        help="Provisional straight entry/exit yaw gate, degrees")


    # --------------------------------------------------------
    # Banner search
    # --------------------------------------------------------

    parser.add_argument(
        "--search-speed",

        type=float,

        default=0.50,
    )
    parser.add_argument("--camera-phase-timeout", type=float, default=120.0,
                        help="Provisional total search/centre/approach deadline, seconds")
    parser.add_argument("--camera-travel-limit", type=float, default=10.0,
                        help="Provisional 3D displacement bound from first valid camera-phase pose")
    parser.add_argument("--camera-max-age", type=float, default=0.5,
                        help="Provisional maximum decoded frame receipt age, seconds")
    parser.add_argument("--camera-processing-width", type=int, default=640,
                        help="Maximum forward-camera processing width; 640 preserves Gazebo baseline")
    parser.add_argument("--no-gui", action="store_true",
                        help="Disable the optional out-of-process camera preview")
    parser.add_argument("--green-hsv-low", type=int, nargs=3, default=(45, 100, 80),
                        metavar=("H", "S", "V"), help="Provisional Gazebo banner HSV lower bound")
    parser.add_argument("--green-hsv-high", type=int, nargs=3, default=(75, 255, 255),
                        metavar=("H", "S", "V"), help="Provisional Gazebo banner HSV upper bound")
    parser.add_argument("--banner-min-area-640x480", type=float, default=700.,
                        help="Banner contour area threshold at 640x480; scales with processing size")
    parser.add_argument("--banner-morph-kernel", type=int, default=5,
                        help="Odd-pixel green-mask opening kernel, Gazebo baseline 5")
    parser.add_argument("--banner-aspect-range", type=float, nargs=2,
                        default=(0.8, 5.0), metavar=("MIN", "MAX"),
                        help="Allowed banner contour aspect ratio during search/centering")
    parser.add_argument("--banner-min-extent", type=float, default=0.15,
                        help="Minimum green contour fill fraction")


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
    if not args.start_mission:
        parser.error("--start-mission is required for arm and takeoff")
    try:
        startup_config = StartupConfig(takeoff_altitude_m=args.takeoff_altitude)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        banner_guard_config = BannerGuardConfig(
            timeout_s=args.camera_phase_timeout,
            max_displacement_m=args.camera_travel_limit,
            frame_max_age_s=args.camera_max_age,
        )
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.entrance_commit_range) or args.entrance_commit_range <= 0:
        parser.error("--entrance-commit-range must be finite and positive")
    if not math.isfinite(args.pre_entry_descent) or args.pre_entry_descent <= 0:
        parser.error("--pre-entry-descent must be finite and positive")
    if args.banner_loss_frames < 1:
        parser.error("--banner-loss-frames must be at least 1")
    try:
        MissionRunnerConfig(
            enter_corridor_distance_m=args.enter_distance,
            vehicle_width_m=args.vehicle_width,
            passage_side_margin_m=args.passage_side_margin,
            enter_max_cross_track_m=args.max_path_cross_track,
            enter_max_yaw_error_deg=args.max_path_yaw_error,
        )
    except ValueError as exc:
        parser.error(str(exc))
    if args.camera_processing_width <= 0:
        parser.error("--camera-processing-width must be positive")
    forward_processing_width = args.camera_processing_width
    if not math.isfinite(args.coverage_max_wall_seconds) or args.coverage_max_wall_seconds <= 0:
        parser.error("--coverage-max-wall-seconds must be finite and positive")
    coverage_cfg = CoverageConfig.load(args.coverage_config)
    return_profile=ReturnConfig.load(args.return_config)
    if args.test_return_only and os.environ.get('MISSION_OWNED_RETURN_FIXTURE')!='1':
        parser.error('Return-only initialization is restricted to the owned Gazebo harness')
    if args.test_field_qr_only:
        if (os.environ.get('MISSION_OWNED_FIELD_QR_FIXTURE')!='1' or
                not args.qr_only or args.coverage_only or args.test_return_only or
                abs(args.takeoff_altitude-coverage_cfg.altitude)>1e-6):
            parser.error('Field QR initialization requires the owned fixture and QR-only endpoint')
    if args.test_field_qr_approach and not args.test_field_qr_only:
        parser.error('Textured approach initialization requires the owned field fixture')
    if args.coverage_only and args.qr_only:
        parser.error('Choose coverage-only or QR-only, not both')
    return_enabled=not (args.coverage_only or args.qr_only)
    if return_enabled:
        stand_off=FieldFrame(coverage_cfg).point(return_profile.stand_off_point)
        if not (coverage_cfg.n_min+coverage_cfg.clearance <= stand_off[0] <= coverage_cfg.n_max-coverage_cfg.clearance
                and coverage_cfg.e_min+coverage_cfg.clearance <= stand_off[1] <= coverage_cfg.e_max-coverage_cfg.clearance):
            parser.error('Return stand-off is outside the registered safe field')
    if not args.coverage_only and not args.test_return_only and not args.test_field_qr_only:
        if abs(args.takeoff_altitude-5.)>1e-6:
            parser.error('QR mission requires the initial 5 m takeoff height')
        decoder_self_check()  # Native/backend failure must be discovered before arming.
    if args.test_field_qr_only:
        decoder_self_check()
    downward_sensors=DownwardSensors(coverage_cfg)
    if coverage_cfg.body_radius < args.vehicle_width / 2:
        parser.error("coverage body_radius is smaller than half corridor vehicle width")



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
        "PREFLIGHT -> GUIDED -> ARM -> TAKEOFF -> BANNER_SEARCH"
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
        f"APPROACH_CORRIDOR -> VERIFIED STAGING at {args.entrance_commit_range:g} m limit -> "
        f"DESCEND {args.pre_entry_descent:g} m -> WALL READINESS"
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

    initial_heartbeat = master.wait_heartbeat(timeout=10)
    if initial_heartbeat is None:
        master.close()
        raise RuntimeError("Flight-controller heartbeat timeout")
    master.target_system = initial_heartbeat.get_srcSystem()
    master.target_component = initial_heartbeat.get_srcComponent()
    telemetry.heartbeat_time = time.monotonic()
    telemetry.mode = mavutil.mode_string_v10(initial_heartbeat)
    telemetry.armed = bool(initial_heartbeat.base_mode &
        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

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

    banner_detector = HybridBannerDetector(
        lower_green=args.green_hsv_low,
        upper_green=args.green_hsv_high,
        min_area_at_640x480=args.banner_min_area_640x480,
        morph_kernel_size=args.banner_morph_kernel,
        aspect_ratio_bounds=args.banner_aspect_range,
        min_extent=args.banner_min_extent,
    )
    preview = PreviewService(enabled=not args.no_gui)
    snapshot_service = SnapshotService()


    # ========================================================
    # STATE
    # ========================================================

    state = ExperimentState.STARTUP


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
    last_metrics = time.monotonic()

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
    coverage_invoked = False
    field_advance_started = 0.0
    field_advance_target = None
    field_advance_heading = None
    field_advance_boot_start = 0.0
    banner_guard = None
    startup = StartupController(time.monotonic(), startup_config)
    staging_envelope = StagingEnvelope()
    readiness = EntranceReadiness()
    staging_started = 0.0
    staging_boot_start = 0.0
    approach_last_confirmed = 0.0
    mission_arm_requested = False
    corridor_home_altitude = None
    qr_reference = None
    traversal_role='outbound'
    return_egress_started=return_egress_boot=0.
    landing_started=landing_boot=0.
    landing_area_verified=False


    print()

    print(
        "[EXPERIMENT] START -> "
        "STARTUP"
    )

    print()

    async_console = AsyncLogStream(sys.stdout)
    sys.stdout = async_console


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

            with sensor_lock:
                source_clock_fault = (camera_clock_gate.failure
                                      or lidar_clock_gate.failure)
            if source_clock_fault and not coverage_started:
                stop_command_service()
                print(f"[SENSOR CLOCK] ABORT: {source_clock_fault}")
                state = ExperimentState.ABORT
                continue

            if state not in (ExperimentState.STARTUP, ExperimentState.ABORT,
                             ExperimentState.COMPLETE, ExperimentState.LANDING) and not coverage_started:
                if (telemetry.authority_revoked or telemetry.mode != "GUIDED"
                        or not telemetry.armed
                        or telemetry.pose_fault is not None
                        or now - telemetry.heartbeat_time > 2.5
                        or now - telemetry.ekf_time > 1.5
                        or not telemetry.ekf_flags & (8 | 16)
                        or now - telemetry.position_time > LIDAR_POSE_MAX_AGE_S
                        or not attitude_valid(current_attitude())):
                    stop_command_service()
                    print("[AUTHORITY] mission motion cancelled: flight/estimator health changed"
                          f"; pose_fault={telemetry.pose_fault}")
                    state = ExperimentState.ABORT
                    continue
                if active_commands is not None and not active_commands.healthy():
                    print(f"[OUTPUT] command worker failed: {active_commands.failure}")
                    state = ExperimentState.ABORT
                    continue
                if state in (ExperimentState.LIDAR_CORRIDOR,
                             ExperimentState.ADVANCE_TO_FIELD, ExperimentState.RETURN_EGRESS):
                    height_ok = (corridor_home_altitude is not None
                                 and telemetry.relative_alt_m is not None
                                 and math.isfinite(telemetry.relative_alt_m)
                                 and 0 <= now - telemetry.relative_alt_time <= 1.0
                                 and abs(telemetry.relative_alt_m
                                         - corridor_home_altitude) <= .30)
                    if not height_ok:
                        print("[ALTITUDE] corridor/roof-clear height envelope lost")
                        state = ExperimentState.ABORT
                        continue

            if state in (ExperimentState.BANNER_SEARCH,
                         ExperimentState.CAMERA_CORRIDOR_CENTER,
                         ExperimentState.APPROACH_CORRIDOR):
                with sensor_lock:
                    frame_stamp = camera_receipt_time
                position = (telemetry.x_m, telemetry.y_m, telemetry.z_m)
                if any(v is None for v in position):
                    position = None
                may_move, failure = banner_guard.check(
                    now, frame_stamp, position, telemetry.position_time,
                    attitude_valid(current_attitude()))
                if not may_move:
                    send_stop(master)
                    centered_frames = 0
                    if failure:
                        print(f"[CAMERA SAFETY] {failure}")
                        request_land(master)
                        state = ExperimentState.ABORT
                        break
                    time.sleep(0.02)
                    continue


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
            # AUTONOMOUS STARTUP
            # =================================================

            if state == ExperimentState.STARTUP:
                previous = startup.state
                feedback=startup_feedback(startup.state_since)
                if not args.coverage_only:
                    down_frames, down_clock, down_error=downward_sensors.snapshot()
                    # Both cameras must be acquiring before autonomous arming.
                    feedback=replace(feedback,camera_time=min(feedback.camera_time,
                        down_frames[-1][1] if down_frames and not down_error else 0.))
                action = startup.step(feedback, now)
                if startup.state != previous:
                    print(f"[STARTUP] {previous} -> {startup.state}")
                if action is not None:
                    request_startup_action(master, action, startup_config.takeoff_altitude_m)
                    if action == "ARM":
                        mission_arm_requested = True
                    print(f"[STARTUP] requested {action}")
                if startup.state == "ABORT":
                    print(f"[STARTUP] ABORT: {startup.reason}")
                    state = ExperimentState.ABORT
                elif startup.state == "READY":
                    telemetry.authority_started = True
                    active_commands = CommandService(
                        lambda vx, vy, vz, yaw, frame='body':
                            transmit_mission_velocity(master, vx, vy, vz, yaw, frame))
                    active_commands.start()
                    if args.test_return_only:
                        if np.linalg.norm(np.array([telemetry.x_m,telemetry.y_m])-return_profile.stand_off_point)>.3:
                            state=ExperimentState.ABORT
                            print('[TEST] Return fixture not at registered stand-off')
                        else:
                            state=ExperimentState.COVERAGE
                            print('[TEST] Fresh autonomous takeoff -> return-only fixture')
                        continue
                    if args.test_field_qr_only:
                        fixture_start=([-18.2,-4.] if args.test_field_qr_approach else [-10.2,-3.8])
                        if np.linalg.norm(np.array([telemetry.x_m,telemetry.y_m])-fixture_start)>.3:
                            state=ExperimentState.ABORT
                            print('[TEST] Field QR fixture not at registered start')
                        else:
                            qr_reference='REF-001'  # Fixture identity, NOT an initial QR validation.
                            state=ExperimentState.COVERAGE
                            print('[TEST] Fresh autonomous takeoff -> field QR fixture; injected reference')
                        continue
                    if not args.coverage_only:
                        state=ExperimentState.INITIAL_QR
                        initial_result={}
                        initial_args=['--config',str(args.coverage_config),'--fly',
                            '--log',str(args.coverage_log.with_name('initial_qr.jsonl')),
                            '--max-wall-seconds','180','--entry-wall-seconds','30']
                        if args.no_gui: initial_args.append('--no-gui')
                        initial_cfg=replace(coverage_cfg,field_origin_n=0.,field_origin_e=0.,
                                            field_yaw=0.,heading=telemetry.yaw_rad,geofence_latlon=None)
                        try:
                            initial_exit=run_coverage(initial_args,master=master,
                                command_service=active_commands,command_token=active_commands.claim(),
                                config_override=initial_cfg,sensors_override=downward_sensors,
                                qr_mode='initial',result_out=initial_result,
                                initial_origin=(round(telemetry.origin_lat_deg*1e7),
                                    round(telemetry.origin_lon_deg*1e7),telemetry.origin_alt_mm)
                                    if telemetry.origin_lat_deg is not None else None)
                        except (Exception,SystemExit) as exc:
                            print(f'[INITIAL QR] runtime failed: {exc}')
                            initial_exit=1
                        # Both successful and failed QR returns revoke the old
                        # startup token before the manager may issue a safe stop.
                        manager_command_token=active_commands.claim()
                        if initial_exit or initial_result.get('state')!='REFERENCE_READY':
                            state=ExperimentState.ABORT
                            print('[INITIAL QR] no reference; corridor entry forbidden')
                            continue
                        qr_reference=initial_result['last_decision']['qr_reference']
                        # Revoke the startup stage and refresh the manager's telemetry
                        # before returning command ownership to camera/corridor logic.
                        if not refresh_manager_after_qr(master):
                            print('[INITIAL QR] telemetry handback failed; corridor entry forbidden')
                            state=ExperimentState.ABORT
                            continue
                        print('[INITIAL QR] reference latched -> BANNER_SEARCH')
                    banner_guard = BannerGuard(time.monotonic(), banner_guard_config)
                    state = ExperimentState.BANNER_SEARCH
                    print("[STARTUP] settled takeoff/reference -> BANNER_SEARCH")
                time.sleep(0.02)
                continue

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
                    cam_seq = (
                        camera_sequence
                    )
                    cam_frame_stamp = camera_receipt_time
                    # Callback replaces the array; detector never mutates it.
                    frame = (latest_forward_frame
                             if cam_seq != last_camera_sequence
                             and latest_forward_frame is not None else None)


                if latest_forward_frame is None:

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
                        detect_banner(banner_detector,
                            frame, debug=preview.enabled
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
                            source_timestamp=cam_frame_stamp,
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


                    if preview.enabled:
                        debug = result["debug_frame"]
                        cv2.putText(debug, "STATE: BANNER_SEARCH", (10, 145),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (255, 255, 255), 2)
                        preview.submit(debug)


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
                    cam_seq = (
                        camera_sequence
                    )
                    cam_frame_stamp = camera_receipt_time
                    frame = (latest_forward_frame
                             if cam_seq != last_camera_sequence
                             and latest_forward_frame is not None else None)


                if latest_forward_frame is None:

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
                        detect_banner(banner_detector,
                            frame, debug=preview.enabled
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
                        ) * 640.0 / frame.shape[1]

                        error_y = float(
                            result[
                                "error_y"
                            ]
                        ) * 480.0 / frame.shape[0]


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
                            source_timestamp=cam_frame_stamp,
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
                            " Panel loss stops approach; only persistent wall geometry grants LiDAR control."
                            )

                            print(
                                "=========================================="
                            )

                            print()


                    if preview.enabled:
                        debug = result["debug_frame"]
                        cv2.putText(debug, "STATE: CAMERA_CORRIDOR_CENTER", (10, 145),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (255, 255, 255), 2)
                        preview.submit(debug)


            # Camera loss is never evidence of arrival.
            elif state == ExperimentState.APPROACH_CORRIDOR:
                with sensor_lock:
                    scan = latest_scan
                    cam_seq = camera_sequence
                    cam_frame_stamp = camera_receipt_time
                    frame = (latest_forward_frame
                             if cam_seq != last_camera_sequence
                             and latest_forward_frame is not None else None)

                if scan is None or scan.age_s > 0.30:
                    send_stop(master)
                    time.sleep(0.02)
                    continue

                front = sector_clearance(scan, half_cone_deg=10.)
                if front is None:
                    send_stop(master)
                    time.sleep(0.02)
                    continue
                if front <= args.entrance_commit_range:
                    send_stop(master)
                    staging_ok = (
                        staging_envelope.contains(
                            telemetry.x_m, telemetry.y_m, telemetry.relative_alt_m)
                        and 0 <= now - telemetry.relative_alt_time <= 1.0
                        and telemetry.relative_alt_m - args.pre_entry_descent
                        >= staging_envelope.min_home_alt_m
                        and attitude_valid(current_attitude())
                        and current_pose() is not None
                        and 0 <= now - approach_last_confirmed <= .5)
                    if staging_ok:
                        descent = None
                        descent_started = now
                        staging_started = now
                        staging_boot_start = telemetry.position_boot_s
                        readiness.reset()
                        state = ExperimentState.DESCEND_BEFORE_PRE_ENTRY
                        print(f"[APPROACH] {front:.2f} m hard limit; verified staging region -> "
                              f"DESCEND {args.pre_entry_descent:g} m, then require wall readiness")
                    elif now - last_diag >= .5:
                        print("[APPROACH] hard limit; holding for recent panel and safe staging pose")
                        last_diag = now
                    continue

                if latest_forward_frame is None:
                    send_stop(master)
                elif cam_seq != last_camera_sequence:
                    last_camera_sequence = cam_seq
                    result = detect_banner(banner_detector, frame,
                                           relaxed_approach=True,
                                           debug=preview.enabled)
                    approach_detection_diag = (
                        f"detected={result['detected']} area={result['area']:.0f}px2 "
                        f"largest={result['largest_area']:.0f}px2 clipped={result['clipped']} "
                        f"reason={result['rejection_reason']}")
                    if result["detected"]:
                        banner_lost_frames = 0
                        approach_last_confirmed = now
                        vy = clamp(float(result["error_x"]) * 640.0 / frame.shape[1]
                                   * args.camera_gain,
                                   -args.camera_max_speed, args.camera_max_speed)
                        vz = clamp(float(result["error_y"]) * 480.0 / frame.shape[0]
                                   * args.camera_gain,
                                   -args.camera_max_speed, args.camera_max_speed)
                        send_camera_velocity(master, abs(args.approach_speed), vy, vz,
                                             source_timestamp=cam_frame_stamp)
                    else:
                        banner_lost_frames += 1
                        send_stop(master)
                        if banner_lost_frames >= args.banner_loss_frames:
                            # Continue stopped and allow bounded reacquisition.
                            # The phase deadline is intentionally not reset.
                            approach_detection_diag += "; holding for reacquisition"
                    preview.submit(result["debug_frame"])

                if now - last_diag >= 0.5:
                    range_text = "unavailable" if front is None else f"{front:.2f}m"
                    print(f"[APPROACH] panel loss frames="
                          f"{banner_lost_frames}/{args.banner_loss_frames} "
                          f"front={range_text} scan_age={scan.age_s:.2f}s "
                          f"{approach_detection_diag}")
                    last_diag = now

            elif state == ExperimentState.DESCEND_BEFORE_PRE_ENTRY:
                if (now - staging_started > 120.0
                        or telemetry.position_boot_s < staging_boot_start
                        or not staging_envelope.contains(
                            telemetry.x_m, telemetry.y_m, telemetry.relative_alt_m)
                        or not 0 <= now - telemetry.relative_alt_time <= 1.0
                        or not attitude_valid(current_attitude())):
                    send_stop(master)
                    print("[STAGING] ABORT: pose, height, attitude, or time envelope lost")
                    state = ExperimentState.ABORT
                    continue
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
                with sensor_lock:
                    staging_scan = latest_scan
                    staging_sequence = scan_sequence
                staging_pose = current_pose()
                staging_attitude = current_attitude()
                if (now - staging_started > 120.0
                        or telemetry.position_boot_s < staging_boot_start
                        or not staging_envelope.contains(
                            telemetry.x_m, telemetry.y_m, telemetry.relative_alt_m)
                        or not 0 <= now - telemetry.relative_alt_time <= 1.0):
                    print("[STAGING] ABORT: staging envelope or deadline lost")
                    state = ExperimentState.ABORT
                    continue
                if (staging_scan is None or staging_scan.age_s > .30
                        or not pose_valid(staging_pose, .5)
                        or not attitude_valid(staging_attitude)):
                    readiness.reset()
                else:
                    readiness.observe(staging_sequence, staging_scan,
                                      staging_attitude, staging_pose)
                wall_ready = (readiness.count >= readiness.required_scans
                              and staging_scan is not None
                              and staging_scan.age_s <= .30
                              and attitude_valid(staging_attitude)
                              and pose_valid(staging_pose, .5))
                age = now - telemetry.position_time
                values = (telemetry.z_m, telemetry.vx_m_s,
                          telemetry.vy_m_s, telemetry.vz_m_s)
                valid = all(v is not None and math.isfinite(v) for v in values)
                if (now - hover_started >= 90.0 or age > 3.5
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
                    if (telemetry.position_boot_s - hover_stable_since >= 2.0
                            and wall_ready):
                        corridor = create_corridor_runner(
                            args.enter_distance, args.vehicle_width,
                            args.passage_side_margin, args.max_path_cross_track,
                            args.max_path_yaw_error)
                        corridor_home_altitude = telemetry.relative_alt_m
                        previous_native_state = None
                        last_scan_sequence = -1
                        state = ExperimentState.LIDAR_CORRIDOR
                        print("[HOVER] settled with persistent fresh two-wall geometry "
                              "-> PRE_ENTRY_GEOMETRY_LOCK")

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
                    scan = None


                # One native FSM iteration per LiDAR scan.
                if (
                    scan is None or scan_seq != last_scan_sequence
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


                    corridor_step_started = time.monotonic()
                    output = (
                        corridor.step(

                            scan=scan,

                            attitude=attitude,

                            pose=pose,
                        )
                    )
                    health_metrics.observe("corridor_step",
                                           time.monotonic() - corridor_step_started)


                    if scan is not None and not preentry_snapshot_saved:
                        try:
                            g = corridor.pre_entry.last_geometry
                            fields = ("confidence", "strict_valid", "loose_valid",
                                      "front_clearance", "width", "left_inliers",
                                      "right_inliers", "left_span", "right_span",
                                      "left_rms", "right_rms", "sectors")
                            geometry = {key: getattr(g, key, None) for key in fields}
                            details = {"lidar_world_pose": scan_world_pose,
                                       "ekf_z": telemetry.z_m, "geometry": geometry}
                            details["local_position_ned"] = {
                                "x": telemetry.x_m, "y": telemetry.y_m,
                                "z": telemetry.z_m, "yaw_rad": telemetry.yaw_rad,
                            }
                            snapshot_service.submit(args.debug_output, scan, details)
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
                        send_native_velocity(
                            master, command,
                            source_timestamp=scan.timestamp if scan is not None else None)


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
                            f"{scan.age_s if scan is not None else float('inf'):.3f}s "

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
                        if traversal_role=='return':
                            return_egress_started=now
                            return_egress_boot=telemetry.position_boot_s
                            state=ExperimentState.RETURN_EGRESS
                            print('[RETURN] Corridor exited; checking exterior landing clearance')
                            continue
                        set_forward_camera_active(node,False)
                        preview.stop()
                        if coverage_cfg.geofence_latlon is not None:
                            try:
                                coverage_cfg=coverage_cfg.register_origin(
                                    telemetry.origin_lat_deg, telemetry.origin_lon_deg)
                            except (ValueError,TypeError) as exc:
                                print(f'[COVERAGE] ABORT: geofence registration: {exc}')
                                state=ExperimentState.ABORT
                                continue
                        # Field starts beyond the corridor roof. This catches
                        # gross spawn/registration errors, not subtle EKF drift.
                        registered = coverage_entry_registered(telemetry, coverage_cfg, now)
                        if not registered:
                            print("[COVERAGE] ABORT: corridor exit outside registered field")
                            state = ExperimentState.ABORT
                        else:
                            field_advance_heading = telemetry.yaw_rad
                            exit_ne = np.array([telemetry.x_m, telemetry.y_m])
                            travel = coverage_cfg.clearance + .4
                            field_advance_target = exit_ne + travel * np.array([
                                math.cos(field_advance_heading),
                                math.sin(field_advance_heading)])
                            target_field = FieldFrame(coverage_cfg).point(field_advance_target)
                            if not (coverage_cfg.n_min+coverage_cfg.clearance <= target_field[0] <= coverage_cfg.n_max-coverage_cfg.clearance
                                    and coverage_cfg.e_min+coverage_cfg.clearance <= target_field[1] <= coverage_cfg.e_max-coverage_cfg.clearance):
                                print('[COVERAGE] ABORT: corridor heading does not lead into field inset')
                                registered = False
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
                fresh = field_advance_health(telemetry, coverage_cfg, now)
                if (now - field_advance_started > 30.0
                        or telemetry.position_boot_s < field_advance_boot_start
                        or telemetry.position_boot_s - field_advance_boot_start > 12.0):
                    send_stop(master)
                    print("[COVERAGE] ABORT: field-entry progress timed out")
                    state = ExperimentState.ABORT
                elif not fresh:
                    send_stop(master)
                    print("[COVERAGE] ABORT: field-entry pose/heading/envelope invalid")
                    state = ExperimentState.ABORT
                elif np.dot(field_advance_target-np.array([telemetry.x_m,telemetry.y_m]),
                            np.array([math.cos(field_advance_heading),math.sin(field_advance_heading)])) > 0:
                    # Advance along the measured corridor exit bearing, not world north.
                    forward, right = heading_velocity_in_body(.15, field_advance_heading,
                                                              telemetry.yaw_rad)
                    send_camera_velocity(master, forward, right, 0.0)
                elif math.hypot(telemetry.vx_m_s, telemetry.vy_m_s) > .10:
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
                if (not field_advance_health(telemetry, coverage_cfg, now)
                        or np.dot(field_advance_target-np.array([telemetry.x_m,telemetry.y_m]),
                                  np.array([math.cos(field_advance_heading),math.sin(field_advance_heading)])) > .1):
                    send_stop(master)
                    print("[COVERAGE] ABORT: clear-to-climb envelope lost")
                    state = ExperimentState.ABORT
                    continue
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
                # Also applies to owned fixtures that skip outbound traversal.
                set_forward_camera_active(node,False)
                preview.stop()
                # The leased sender remains the one navigation output owner.
                # Claiming coverage authority revokes all corridor proposals.
                send_stop(master)
                coverage_token = active_commands.claim()
                print("[TIMING PRE-COVERAGE]", health_metrics.snapshot_and_reset(),
                      "preview_dropped=", preview.dropped)
                coverage_started = True
                coverage_invoked = True
                try:
                    coverage_args = [
                        "--config", str(args.coverage_config),
                        "--log", str(args.coverage_log),
                        "--max-wall-seconds", str(args.coverage_max_wall_seconds),
                        "--entry-wall-seconds", "30",
                        "--fly",
                    ]
                    if args.no_gui:
                        coverage_args.append('--no-gui')
                    coverage_result={}
                    orange_approach=(ReturnApproach(coverage_cfg,return_profile,args,
                        return_sensor_snapshot,preview,
                        activate=lambda: set_forward_camera_active(node,True)) if return_enabled else None)
                    coverage_exit = run_coverage(coverage_args, master=master,
                        command_service=active_commands, command_token=coverage_token,
                        config_override=coverage_cfg,
                        sensors_override=downward_sensors,
                        qr_mode=('return_test' if args.test_return_only else
                                 None if args.coverage_only else 'field'),
                        reference=qr_reference,
                        result_out=coverage_result,
                        return_profile=return_profile if return_enabled else None,
                        return_approach=orange_approach,
                        initial_origin=(round(telemetry.origin_lat_deg*1e7),
                                        round(telemetry.origin_lon_deg*1e7),
                                        telemetry.origin_alt_mm)
                        if coverage_cfg.geofence_latlon is not None else None)
                except (Exception, SystemExit) as exc:
                    # Coverage may already have commanded motion. Do not reclaim
                    # authority using the corridor's now-stale telemetry.
                    print(f"[COVERAGE] runtime failed: {exc}")
                    state = ExperimentState.ABORT
                else:
                    coverage_token=coverage_result.get('command_token',coverage_token)
                    if coverage_exit==0 and coverage_result.get('state')=='RETURN_ENTRY_READY':
                        manager_command_token=active_commands.claim()
                        if not refresh_manager_after_qr(master):
                            state=ExperimentState.ABORT
                        else:
                            yaw_error=math.atan2(math.sin(telemetry.yaw_rad-return_profile.heading),
                                                 math.cos(telemetry.yaw_rad-return_profile.heading))
                            if (not return_profile.approach_contains(
                                    [telemetry.x_m,telemetry.y_m],telemetry.relative_alt_m)
                                    or abs(yaw_error)>math.radians(15)
                                    or orange_approach.target_alt is None
                                    or abs(telemetry.relative_alt_m-orange_approach.target_alt)>.15):
                                state=ExperimentState.ABORT
                                print('[RETURN] Fresh handoff pose/height not in staged envelope')
                                continue
                            corridor=create_corridor_runner(args.enter_distance,args.vehicle_width,
                                args.passage_side_margin,args.max_path_cross_track,args.max_path_yaw_error)
                            corridor_home_altitude=telemetry.relative_alt_m
                            traversal_role='return'; previous_native_state=None
                            last_scan_sequence=-1; preentry_snapshot_saved=True
                            coverage_started=False
                            state=ExperimentState.LIDAR_CORRIDOR
                            print('[RETURN] Fresh read/command handoff -> native corridor FSM')
                    else:
                        state = (ExperimentState.COMPLETE if coverage_exit == 0 and not return_enabled
                                 else ExperimentState.ABORT)
                    print(f"[MISSION SEGMENT] {coverage_result.get('state','ABORT')}")

            elif state == ExperimentState.RETURN_EGRESS:
                axis=return_profile.axis
                delta=np.array([telemetry.x_m,telemetry.y_m])-return_profile.entrance
                along=float(delta@axis)
                across=float(delta@np.array([-axis[1],axis[0]]))
                elapsed=telemetry.position_boot_s-return_egress_boot
                target=return_profile.far_mouth_distance+return_profile.landing_clearance
                yaw_error=math.atan2(math.sin(telemetry.yaw_rad-return_profile.heading),
                                     math.cos(telemetry.yaw_rad-return_profile.heading))
                with sensor_lock: egress_scan=latest_scan
                front=(sector_clearance(egress_scan,half_cone_deg=18.)
                       if egress_scan is not None and egress_scan.age_s<=.30 else None)
                if (elapsed<0 or elapsed>30 or now-return_egress_started>120
                        or abs(across)>return_profile.entry_half_width
                        or not return_profile.far_mouth_distance-1 <= along <= target+.8
                        or abs(yaw_error)>math.radians(15) or front is None or front<.6):
                    send_stop(master); state=ExperimentState.ABORT
                    print('[RETURN] Egress geometry, clearance or progress invalid')
                elif along<target:
                    vx,vy=heading_velocity_in_body(.15,return_profile.heading,telemetry.yaw_rad)
                    send_camera_velocity(master,vx,vy,0.)
                elif max(abs(telemetry.vx_m_s),abs(telemetry.vy_m_s),abs(telemetry.vz_m_s))>.10:
                    send_stop(master)
                else:
                    landing_area_verified=True
                    telemetry.landing_expected=True
                    landing_started=now; landing_boot=telemetry.position_boot_s
                    request_land(master)
                    if confirm_terminal_land(master):
                        state=ExperimentState.LANDING
                        print('[RETURN] Exterior landing region verified; FC LAND confirmed')
                    else:
                        state=ExperimentState.ABORT
                        print('[RETURN] LAND mode was not confirmed')

            elif state == ExperimentState.LANDING:
                elapsed=telemetry.position_boot_s-landing_boot
                fresh=(now-telemetry.heartbeat_time<=2.5 and now-telemetry.landed_time<=1.5
                       and now-telemetry.relative_alt_time<=1. and now-telemetry.position_time<=1.)
                landed=landing_confirmed_feedback(telemetry,now,landing_started,return_profile)
                if landed:
                    state=ExperimentState.COMPLETE
                    print('[RETURN] COMPLETE: fresh on-ground and disarmed confirmation')
                    args.coverage_log.with_suffix('.landing.json').write_text(
                        json.dumps({'state':'COMPLETE','traversal_role':traversal_role,
                            'n':telemetry.x_m,'e':telemetry.y_m,'home_alt':telemetry.relative_alt_m,
                            'boot_s':telemetry.position_boot_s,'armed':False,
                            'landed_state':telemetry.landed_state},indent=2)+'\n')
                elif (not fresh or telemetry.authority_revoked or telemetry.mode!='LAND'
                        or elapsed<0 or elapsed>return_profile.landing_seconds
                        or now-landing_started>180):
                    state=ExperimentState.ABORT
                    print('[RETURN] Landing monitoring failed; touchdown unconfirmed')


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

            if preview.escape_pressed():

                print(
                    "[EXPERIMENT] "
                    "ESC pressed"
                )
                state = ExperimentState.ABORT
                break


            # =================================================
            # LOOP RATE
            # =================================================

            elapsed = (
                time.monotonic()
                -
                loop_started
            )
            if not coverage_invoked:
                health_metrics.observe("mission_loop", elapsed)
            if not coverage_invoked and time.monotonic() - last_metrics >= 5.0:
                print("[TIMING]", health_metrics.snapshot_and_reset(),
                      "preview_dropped=", preview.dropped,
                      "preview_alive=", preview.alive,
                      "preview_exitcode=", preview.exitcode,
                      "console_dropped=", async_console.dropped_lines,
                      "camera_duplicates=", camera_clock_gate.duplicates,
                      "lidar_duplicates=", lidar_clock_gate.duplicates,
                      "command_expirations=",
                      active_commands.expirations if active_commands else 0,
                      "command_send_peak_ms=",
                      round(active_commands.send_duration_peak_s * 1000, 2)
                      if active_commands else 0,
                      "command_deadline_misses=",
                      active_commands.deadline_misses if active_commands else 0,
                      "decision_to_send_peak_ms=",
                      round(active_commands.decision_to_send_peak_s * 1000, 2)
                      if active_commands else 0,
                      "observation_to_send_peak_ms=",
                      round(active_commands.observation_to_send_peak_s * 1000, 2)
                      if active_commands else 0)
                last_metrics = time.monotonic()

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

    except Exception as exc:
        print(f"[EXPERIMENT] exception: {exc}")
        state = ExperimentState.ABORT


    finally:

        print()

        if coverage_started:
            try:
                if active_commands is not None:
                    active_commands.publish(0.,0.,0.,coverage_cfg.heading+coverage_cfg.field_yaw,
                                            token=coverage_token, frame='local')
                stop_command_service()
            except Exception as exc:
                print(f"[SAFETY] coverage output shutdown failed: {exc}")
        else:
            try:
                stop_command_service()
            except Exception as exc:
                print(f"[SAFETY] output worker shutdown failed: {exc}")
            if telemetry.mode == "GUIDED" and not telemetry.authority_revoked:
                try:
                    send_stop(master)
                except Exception as exc:
                    print(f"[SAFETY] STOP dispatch failed: {exc}")
            if (state == ExperimentState.ABORT and mission_arm_requested
                    and (traversal_role!='return' or landing_area_verified)
                    and telemetry.armed and telemetry.mode in ("GUIDED", "LAND")
                    and not (telemetry.authority_revoked and telemetry.mode != "LAND")):
                try:
                    confirmed = confirm_terminal_land(master)
                    print(f"[SAFETY] LAND confirmation: {confirmed}")
                except Exception as exc:
                    print(f"[SAFETY] LAND dispatch/confirmation failed: {exc}")
            elif state==ExperimentState.ABORT and traversal_role=='return' and not landing_area_verified:
                print('[SAFETY] Return aborted: stopped; unverified/roof-covered landing forbidden. FC/operator contingency required.')
        if coverage_started:
            print("[EXPERIMENT] coverage authority released; command sender stopped")

        master.close()

        preview.stop()
        snapshot_service.join()
        if snapshot_service.error is not None:
            print(f"[PRE_ENTRY SNAPSHOT] write failed: {snapshot_service.error}")

        print(
            "[EXPERIMENT] "
            "manager stopped"
        )
        sys.stdout = async_console.stream
        async_console.stop()

    return 0 if state == ExperimentState.COMPLETE else 1


if __name__ == "__main__":
    raise SystemExit(main())
