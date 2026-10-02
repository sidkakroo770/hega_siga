#!/usr/bin/env python3
"""Isolated Gazebo scan inspection and native corridor flight pipeline."""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np
from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan
from pymavlink import mavutil

sys.path.insert(0, str(Path.home() / 'sae_mission2/corridor'))
from native.common.types import NativeScan, Attitude, BodyVelocity, VehicleAction
from native.mission_runner import NativeMissionRunner, MissionRunnerConfig, VehiclePose
from replay import geometry_report


DEFAULT_TOPIC = '/corridor_test/lidar/scan'
lock = threading.Lock()
latest = None
sequence = 0


def receive_scan(msg):
    global latest, sequence
    ranges = np.asarray(msg.ranges, dtype=float)
    if ranges.size != msg.count or msg.vertical_count > 1:
        return
    scan = NativeScan(msg.angle_min + np.arange(ranges.size) * msg.angle_step,
                      ranges, np.zeros_like(ranges), time.monotonic(),
                      msg.range_min, msg.range_max)
    with lock:
        latest = scan
        sequence += 1


def send_velocity(master, command):
    # Native FLU -> MAVLink FRD: negate left, up and CCW yaw rate.
    master.mav.set_position_target_local_ned_send(
        int(time.monotonic()*1000) & 0xffffffff,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
        0, 0, 0, command.vx_m_s, -command.vy_m_s, -command.vz_m_s,
        0, 0, 0, 0, -command.yaw_rate_rad_s)


def request_streams(master):
    for message in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
        master.mav.command_long_send(master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message, 50000, 0, 0, 0, 0, 0)


def create_runner():
    # SIMULATION ONLY: these generous wall-clock limits accommodate slow Gazebo.
    # Review speed, clearances and deadlines before any real-aircraft use.
    # The vehicle starts 2 m before the entrance.  Travel those 2 m plus the
    # normal 0.75 m entry margin before allowing CORRIDOR_CRUISE.
    runner = NativeMissionRunner(MissionRunnerConfig(enter_corridor_distance_m=2.75,
        pre_entry_hold_timeout_s=60, enter_corridor_timeout_s=60,
        reassess_hard_timeout_s=65))
    runner.pre_entry.config.acquire_timeout_s = 60
    runner.pre_entry.config.alignment_timeout_s = 90
    runner.reassess.config.recovery_timeout_s = 60
    runner.exit.config.exit_hard_timeout_s = 90
    return runner


def save_scan(scan, directory):
    directory.mkdir(parents=True, exist_ok=True)
    np.savez(directory / 'live_scan.npz', angles_rad=scan.angles_rad,
             ranges_m=scan.ranges_m, range_min_m=scan.range_min_m,
             range_max_m=scan.range_max_m)
    report, _ = geometry_report(scan)
    (directory / 'live_geometry.json').write_text(json.dumps(report, indent=2))
    print('[GEOMETRY]', json.dumps(report), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fly', action='store_true', help='Send native FSM velocity commands')
    parser.add_argument('--mavlink', default='udpin:0.0.0.0:14552')
    parser.add_argument('--topic', default=DEFAULT_TOPIC,
                        help='Gazebo LaserScan topic (read-only mode supports any scan topic)')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'artifacts')
    args = parser.parse_args()
    node = Node()
    if not node.subscribe(LaserScan, args.topic, receive_scan):
        raise SystemExit('LiDAR subscription failed')
    print(f'[TEST] {"FLIGHT" if args.fly else "READ-ONLY"} topic={args.topic}', flush=True)
    start = time.monotonic()
    while latest is None:
        if time.monotonic() - start > 20:
            raise SystemExit('No scan: check world, partition and publisher')
        time.sleep(.05)
    if not args.fly:
        report = save_scan(latest, args.output)
        print('[INSPECT] No MAVLink connection or movement commands were made.')
        raise SystemExit(0 if report['strict_valid'] else 2)

    master = mavutil.mavlink_connection(args.mavlink, source_system=254)
    print('[MAVLINK] Waiting for autopilot heartbeat...', flush=True)
    deadline = time.monotonic() + 30
    hb = None
    while time.monotonic() < deadline:
        candidate = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if candidate and candidate.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            hb = candidate
            break
    if hb is None:
        raise SystemExit('No ArduPilot heartbeat')
    master.target_system = hb.get_srcSystem()
    master.target_component = hb.get_srcComponent()
    request_streams(master)
    heartbeat_time = time.monotonic()
    position = attitude = None
    position_time = attitude_time = 0.
    last_sequence = -1
    last_print = last_request = 0.
    runner = None
    command = BodyVelocity.stop()
    command_time = 0.
    missing_since = None
    hold_z = None
    started = time.monotonic()
    try:
        while time.monotonic() - started < 900:
            now = time.monotonic()
            for _ in range(500):
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_srcSystem() != master.target_system:
                    continue
                kind = msg.get_type()
                if kind == 'HEARTBEAT' and msg.get_srcComponent() == master.target_component:
                    hb, heartbeat_time = msg, now
                elif kind == 'LOCAL_POSITION_NED':
                    position, position_time = msg, now
                elif kind == 'ATTITUDE':
                    attitude, attitude_time = msg, now
            if now - last_request >= 2:
                request_streams(master); last_request = now
            if now-heartbeat_time > 3 or not (hb.base_mode & 128) or hb.custom_mode != 4:
                print('[STOP] Requires armed GUIDED; take off before starting --fly')
                break
            with lock:
                scan, seq = latest, sequence
            valid = (scan is not None and scan.age_s <= .5 and position is not None
                     and attitude is not None and now-position_time <= .5
                     and now-attitude_time <= .5)
            if valid:
                valid = all(math.isfinite(v) for v in (
                    position.x, position.y, position.z, position.vx, position.vy,
                    position.vz, attitude.roll, attitude.pitch, attitude.yaw))
            if not valid:
                send_velocity(master, BodyVelocity.stop())
                command = BodyVelocity.stop()
                if missing_since is None:
                    missing_since = now
                if now-missing_since > 5:
                    print('[STOP] Scan or pose telemetry unavailable for 5 seconds')
                    break
                time.sleep(.05)
                continue
            missing_since = None
            if runner is None:
                if not .8 <= -position.z <= 1.8 or abs(position.vz) > .1:
                    print('[STOP] Start after takeoff 1.3 has settled (height 0.8–1.8 m)')
                    break
                hold_z = position.z
                save_scan(scan, args.output)
                runner = create_runner()
            if seq != last_sequence:
                last_sequence = seq
                pose = VehiclePose(position.x, position.y, attitude.yaw, position_time)
                imu = Attitude(attitude.roll, attitude.pitch, attitude.yaw, attitude_time)
                output = runner.step(scan, attitude=imu, pose=pose)
                state = runner.public_state().value
                if output.action == VehicleAction.LAND or state == 'ABORT_CORRIDOR':
                    print(f'[FAIL] {state}: {output.reason}; requesting LAND')
                    send_velocity(master, BodyVelocity.stop())
                    master.set_mode('LAND')
                    return
                if state == 'CORRIDOR_EXITED':
                    send_velocity(master, BodyVelocity.stop())
                    print('[PASS] CORRIDOR_EXITED — vehicle holding; use mode land in MAVProxy')
                    return
                # A single vertical authority holds the settled takeoff height.
                command = replace(output.command,
                    vz_m_s=max(-.15, min(.15, position.z-hold_z)))
                command_time = now
                if now-last_print >= .5:
                    g = runner.pre_entry.last_geometry
                    print(f'[{state}] status={output.status} confidence={output.confidence} '
                          f'cmd={command} pose_age={now-position_time:.2f}s '
                          f'scan_age={scan.age_s:.2f}s', flush=True)
                    if state == 'PRE_ENTRY_GEOMETRY_LOCK' and g:
                        print(f'  width={g.width:.2f} inliers={g.left_inliers}/{g.right_inliers} '
                              f'spans={g.left_span:.2f}/{g.right_span:.2f} sectors={g.sectors}')
                    last_print = now
            send_velocity(master, command if now-command_time < .5 else BodyVelocity.stop())
            time.sleep(.05)
        else:
            print('[STOP] 900 second wall-time watchdog')
    except KeyboardInterrupt:
        print('[STOP] Keyboard interrupt')
    finally:
        for _ in range(5):
            send_velocity(master, BodyVelocity.stop())
            time.sleep(.05)
        master.close()


if __name__ == '__main__':
    main()
