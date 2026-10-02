#!/usr/bin/env python3
import os
import sys

system_paths = [
    '/usr/lib/python3/dist-packages',
    '/usr/local/lib/python3/dist-packages',
    '/usr/lib/python3/site-packages'
]
for p in system_paths:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

os.environ['GLOG_minloglevel'] = '2'

import time
import math
import cv2
import numpy as np
import importlib
from pymavlink import mavutil

# --- GAZEBO BINDING LOADER ---
Node = None
Image = None

for t_name in ['gz.transport15', 'gz.transport14', 'gz.transport13', 'gz.transport12', 'gz.transport']:
    try:
        mod = importlib.import_module(t_name)
        if hasattr(mod, 'Node'):
            Node = getattr(mod, 'Node')
            break
    except Exception:
        continue

for m_name in ['gz.msgs12.image_pb2', 'gz.msgs11.image_pb2', 'gz.msgs10.image_pb2', 'gz.msgs.image_pb2']:
    try:
        mod = importlib.import_module(m_name)
        if hasattr(mod, 'Image'):
            Image = getattr(mod, 'Image')
            break
    except Exception:
        continue

if Node is None or Image is None:
    print("[ERROR] Gazebo Transport bindings missing.")
    sys.exit(1)

down_frame = None
front_frame = None

def down_callback(msg):
    global down_frame
    try:
        h, w = msg.height, msg.width
        img_array = np.frombuffer(msg.data, dtype=np.uint8)
        if len(img_array) == h * w * 3:
            frame = img_array.reshape((h, w, 3))
            down_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif len(img_array) == h * w * 4:
            frame = img_array.reshape((h, w, 4))
            down_frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    except Exception:
        pass

def front_callback(msg):
    global front_frame
    try:
        h, w = msg.height, msg.width
        img_array = np.frombuffer(msg.data, dtype=np.uint8)
        if len(img_array) == h * w * 3:
            frame = img_array.reshape((h, w, 3))
            front_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif len(img_array) == h * w * 4:
            frame = img_array.reshape((h, w, 4))
            front_frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    except Exception:
        pass

qr_decoder = cv2.QRCodeDetector()

def detect_full_qr_cluster(frame):
    h, w = frame.shape[:2]
    success, bbox, data = qr_decoder.detectAndDecode(frame)
    if success and bbox is not None and len(bbox) > 0:
        pts = np.int32(bbox[0]) if len(bbox.shape) == 3 else np.int32(bbox)
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        return True, cx, cy, pts, data if data else "Decoded QR"

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(gray, maxCorners=200, qualityLevel=0.05, minDistance=3)
    if corners is None or len(corners) < 6:
        return False, None, None, None, ""

    corners = np.int32(corners).reshape(-1, 2)
    grid_size = 60
    best_cluster, max_count = [], 0

    for x in range(0, w - grid_size, 20):
        for y in range(0, h - grid_size, 20):
            in_win = [pt for pt in corners if x <= pt[0] <= x + grid_size and y <= pt[1] <= y + grid_size]
            if len(in_win) > max_count:
                max_count = len(in_win)
                best_cluster = in_win

    if max_count >= 6:
        c_arr = np.array(best_cluster)
        seed_cx, seed_cy = np.mean(c_arr[:, 0]), np.mean(c_arr[:, 1])
        cluster_points = [pt for pt in corners if np.hypot(pt[0] - seed_cx, pt[1] - seed_cy) < 70]
        if cluster_points:
            pts_arr = np.array(cluster_points)
            min_x, min_y = np.min(pts_arr[:, 0]), np.min(pts_arr[:, 1])
            max_x, max_y = np.max(pts_arr[:, 0]), np.max(pts_arr[:, 1])
            cx, cy = int((min_x + max_x) / 2), int((min_y + max_y) / 2)
            box_pts = np.array([[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]], dtype=np.int32)
            return True, cx, cy, box_pts, "QR Full Block"

    return False, None, None, None, ""

def find_thin_green_gate(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 70, 70])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [c for c in contours if cv2.contourArea(c) > 40]
    
    if valid_contours:
        largest = max(valid_contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return True, cx, cy, largest
    return False, None, None, None

def main():
    global down_frame, front_frame
    print("[INFO] Connecting MAVLink (udp:127.0.0.1:14550)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print("[INFO] Vehicle MAVLink Connected!")

    node = Node()
    node.subscribe(Image, "/iris/camera_downward/image_raw", down_callback)
    node.subscribe(Image, "/iris/camera_forward/image_raw", front_callback)

    def set_mode(mode_name):
        mode_id = master.mode_mapping()[mode_name]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

    def arm_and_takeoff(target_alt):
        print(f"[MISSION] Setting GUIDED mode and Arming...")
        set_mode("GUIDED")
        time.sleep(1)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        print(f"[MISSION] Executing Takeoff to {target_alt}m...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_alt
        )

    def send_velocity_frame(vx, vy, vz, yaw_deg=0.0):
        yaw_rad = math.radians(yaw_deg)
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000100111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            yaw_rad, 0.0
        )

    def stream_velocity_body(vx, vy, vz, duration_sec, yaw_deg=0.0):
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            send_velocity_frame(vx, vy, vz, yaw_deg)
            time.sleep(0.1)

    def set_yaw_deg(heading_deg=0.0):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading_deg,
            0,
            1,
            0,
            0, 0, 0
        )

    def get_altitude():
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
        return abs(msg.z) if msg else 0.0

    # Step 1: Takeoff to 10m Altitude
    arm_and_takeoff(10.0)
    while get_altitude() < 9.5:
        time.sleep(0.5)
    print("[MISSION] Reached 10m Altitude!")

    # Step 2: Move Forward 1m
    print("[MISSION] Moving position 1m forward...")
    stream_velocity_body(0.5, 0.0, 0.0, duration_sec=2.0, yaw_deg=0.0)
    stream_velocity_body(0.0, 0.0, 0.0, duration_sec=1.0, yaw_deg=0.0)

    # Step 3: Downward Camera QR Centering
    KP = 0.002
    TOLERANCE_PX = 25
    print("[MISSION] Waiting for downward camera feed...")
    while down_frame is None:
        time.sleep(0.05)

    cv2.namedWindow("Downward QR Mission", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Front Gate Inspector", cv2.WINDOW_NORMAL)

    print("[MISSION] Active QR Target Centering (Downward View)...")
    centered = False
    while not centered:
        frame = down_frame.copy()
        h, w, _ = frame.shape
        img_cx, img_cy = w // 2, h // 2

        found, qr_x, qr_y, box_pts, label = detect_full_qr_cluster(frame)
        if found:
            err_x, err_y = qr_x - img_cx, qr_y - img_cy
            cv2.polylines(frame, [box_pts], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.drawMarker(frame, (qr_x, qr_y), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
            
            vx = -err_y * KP
            vy = err_x * KP
            send_velocity_frame(vx, vy, 0.0, yaw_deg=0.0)

            if abs(err_x) < TOLERANCE_PX and abs(err_y) < TOLERANCE_PX:
                print("[MISSION] QR Target Centered!")
                stream_velocity_body(0.0, 0.0, 0.0, duration_sec=1.0, yaw_deg=0.0)
                centered = True
        else:
            send_velocity_frame(0.0, 0.0, 0.0, yaw_deg=0.0)

        cv2.imshow("Downward QR Mission", frame)
        cv2.waitKey(1)

    # Step 4: Move -30m Backward (-1.5 m/s for 20s)
    print("[MISSION] Moving drone -30m backward...")
    stream_velocity_body(-1.5, 0.0, 0.0, duration_sec=20.0, yaw_deg=0.0)
    stream_velocity_body(0.0, 0.0, 0.0, duration_sec=1.0, yaw_deg=0.0)

    # Step 4b: Force set_yaw(0, 0, 0)
    print("[MISSION] Executing set_yaw(0, 0, 0) to enforce heading 0°...")
    set_yaw_deg(0.0)
    time.sleep(2.0)

    # Step 5: Descend to 2.5m Altitude
    print("[MISSION] Descending to 2.5m altitude...")
    while get_altitude() > 2.6:
        send_velocity_frame(0.0, 0.0, 0.5, yaw_deg=0.0)
        time.sleep(0.1)
    stream_velocity_body(0.0, 0.0, 0.0, duration_sec=1.0, yaw_deg=0.0)
    print("[MISSION] Reached 2.5m Altitude!")

    # Step 6: Detect Thin Green Gate Frame via Front Camera
    print("[MISSION] Aligning with Thin Green Gate Frame...")
    aligned_gate = False
    while not aligned_gate:
        if front_frame is None:
            time.sleep(0.05)
            continue

        fframe = front_frame.copy()
        fh, fw, _ = fframe.shape
        f_cx = fw // 2

        g_found, g_x, g_y, g_contour = find_thin_green_gate(fframe)
        if g_found:
            err_x = g_x - f_cx
            cv2.drawContours(fframe, [g_contour], -1, (0, 255, 0), 2)
            cv2.drawMarker(fframe, (g_x, g_y), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
            cv2.putText(fframe, f"LOCKED GREEN GATE [ERR:{err_x}]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            vy = err_x * 0.0015
            send_velocity_frame(0.0, vy, 0.0, yaw_deg=0.0)

            if abs(err_x) < 25:
                print("[MISSION] Aligned with Green Gate!")
                stream_velocity_body(0.0, 0.0, 0.0, duration_sec=1.0, yaw_deg=0.0)
                aligned_gate = True
        else:
            send_velocity_frame(0.0, 0.0, 0.0, yaw_deg=0.0)
            cv2.putText(fframe, "SEARCHING THIN GREEN GATE...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("Front Gate Inspector", fframe)
        cv2.waitKey(1)

    # Step 7: Move Full 30m Forward through the Gate (1.5 m/s for 20s)
    print("[MISSION] Flying FULL 30m Forward through Green Gate...")
    stream_velocity_body(1.5, 0.0, 0.0, duration_sec=20.0, yaw_deg=0.0)
    stream_velocity_body(0.0, 0.0, 0.0, duration_sec=1.0, yaw_deg=0.0)

    # Step 8: Ascend back to 10m Altitude
    print("[MISSION] Ascending back to 10m Altitude...")
    while get_altitude() < 9.5:
        send_velocity_frame(0.0, 0.0, -0.5, yaw_deg=0.0)
        time.sleep(0.1)
    stream_velocity_body(0.0, 0.0, 0.0, duration_sec=1.0, yaw_deg=0.0)

    # Step 9: Return to Launch (RTL)
    print("[MISSION] Mission Complete! Executing Return to Launch (RTL)...")
    set_mode("RTL")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
