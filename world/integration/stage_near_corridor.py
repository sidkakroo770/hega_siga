#!/usr/bin/env python3

import time
import math
from pymavlink import mavutil


CONNECTION = "udp:127.0.0.1:14550"


print("========================================")
print(" MISSION 2 CORRIDOR STAGING TEST")
print("========================================")
print()

master = mavutil.mavlink_connection(CONNECTION)
print("[MAVLINK] waiting for heartbeat...")
master.wait_heartbeat()
print("[MAVLINK] connected")


def set_mode(name):
    mapping = master.mode_mapping()

    if name not in mapping:
        raise RuntimeError(f"Mode {name} unavailable")

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[name],
    )

    time.sleep(1.0)


def send_body_velocity(vx, vy, vz, yaw_deg=0.0):
    """
    Same BODY_NED velocity interface used by coworker's mission.
    """

    yaw_rad = math.radians(yaw_deg)

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000100111000111,

        # position ignored
        0, 0, 0,

        # velocity
        vx, vy, vz,

        # acceleration ignored
        0, 0, 0,

        # yaw / yaw-rate
        yaw_rad,
        0.0,
    )


def stream_velocity(vx, vy, vz, seconds, yaw_deg=0.0):
    end = time.time() + seconds

    while time.time() < end:
        send_body_velocity(
            vx,
            vy,
            vz,
            yaw_deg,
        )

        time.sleep(0.1)


def get_local_position(timeout=1.0):
    msg = master.recv_match(
        type="LOCAL_POSITION_NED",
        blocking=True,
        timeout=timeout,
    )

    if msg is None:
        return None

    return (
        float(msg.x),
        float(msg.y),
        float(msg.z),
    )


def print_position(label):
    p = get_local_position()

    if p is None:
        print(f"[POSE] {label}: unavailable")
        return

    n, e, d = p

    print(
        f"[POSE] {label}: "
        f"N={n:+.2f} "
        f"E={e:+.2f} "
        f"D={d:+.2f}"
    )


def altitude():
    p = get_local_position()

    if p is None:
        return None

    return -p[2]


def set_yaw(heading_deg):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading_deg,
        0,
        1,
        0,
        0,
        0,
        0,
    )


print_position("initial")


# ------------------------------------------------------------
# GUIDED + ARM
# ------------------------------------------------------------

print("[MISSION] GUIDED")
set_mode("GUIDED")

print("[MISSION] ARM")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
)

time.sleep(2.0)


# ------------------------------------------------------------
# Takeoff 10 m
# ------------------------------------------------------------

print("[MISSION] takeoff -> 10 m")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    10.0,
)

while True:
    alt = altitude()

    if alt is not None:
        print(
            f"\r[ALT] {alt:.2f} m",
            end="",
            flush=True,
        )

        if alt >= 9.5:
            break

    time.sleep(0.5)

print()
print_position("after takeoff")


# ------------------------------------------------------------
# Coworker Step 2
# ------------------------------------------------------------

print("[MISSION] forward ~1 m")
stream_velocity(
    0.5,
    0.0,
    0.0,
    2.0,
)

stream_velocity(
    0.0,
    0.0,
    0.0,
    1.0,
)

print_position("after forward 1 m")


# ------------------------------------------------------------
# QR centering intentionally skipped.
# ------------------------------------------------------------

print("[MISSION] QR centering SKIPPED for staging test")


# ------------------------------------------------------------
# Coworker Step 4
# ------------------------------------------------------------

print("[MISSION] backward ~30 m in BODY frame")

stream_velocity(
    -1.5,
    0.0,
    0.0,
    20.0,
)

stream_velocity(
    0.0,
    0.0,
    0.0,
    1.0,
)

print_position("after backward 30 m")


# ------------------------------------------------------------
# Coworker Step 4b
# ------------------------------------------------------------

print("[MISSION] yaw -> 0 deg")
set_yaw(0.0)
time.sleep(3.0)

print_position("after yaw 0")


# ------------------------------------------------------------
# Coworker Step 5
# BODY_NED +Z is DOWN.
# ------------------------------------------------------------

print("[MISSION] descending -> 2.5 m")

while True:
    alt = altitude()

    if alt is None:
        continue

    print(
        f"\r[ALT] {alt:.2f} m",
        end="",
        flush=True,
    )

    if alt <= 2.6:
        break

    send_body_velocity(
        0.0,
        0.0,
        +0.5,
        0.0,
    )

    time.sleep(0.1)

stream_velocity(
    0.0,
    0.0,
    0.0,
    2.0,
)

print()
print_position("CORRIDOR STAGING POSITION")

print()
print("========================================")
print(" STAGING COMPLETE")
print(" Vehicle remains in GUIDED hover.")
print(" Now run test_fsm_readonly.py")
print("========================================")
