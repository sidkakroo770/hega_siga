"""
SkyScan Boustrophedon & Optical Wall-Following
Author: Thakur Dhruv Singh
Roll Number: 1024150354
"""

import sys, time, math
import cv2
import numpy as np
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# --- GEOFENCE BOUNDARIES & PARAMS ---
X_MIN, X_MAX = -19.3, 17.9
Y_MIN, Y_MAX = -13.4, 14.3
TOLERANCE = 1.5    

# --- DYNAMIC VELOCITY PARAMS ---
MAX_SPEED = 1.5      
TRACE_SPEED = 0.4    
EVADE_SPEED = 0.3    

# --- OPENCV SETUP ---
LOWER_RED_1, UPPER_RED_1 = np.array([0, 120, 70]), np.array([10, 255, 255])
LOWER_RED_2, UPPER_RED_2 = np.array([170, 120, 70]), np.array([180, 255, 255])

class CameraViewer:
    def __init__(self):
        self.node = Node()
        self.latest_frame = None
        topic = "/iris/camera_downward/image_raw"
        print(f"[CAM1] Subscribing to active image topic: {topic}")
        self.node.subscribe(Image, topic, self.image_callback)

    def image_callback(self, msg: Image):
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.pixel_format_type == 3: 
            frame = img_data.reshape((msg.height, msg.width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.pixel_format_type == 1: 
            frame = img_data.reshape((msg.height, msg.width))
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            frame = img_data.reshape((msg.height, msg.width, -1))
        self.latest_frame = frame 

class Telemetry:
    def __init__(self):
        self.x, self.y, self.z, self.yaw, self.alt = 0.0, 0.0, 0.0, 0.0, 0.0

    def update(self, master):
        while True:
            msg = master.recv_match(blocking=False)
            if not msg: break
            msg_type = msg.get_type()
            if msg_type == 'LOCAL_POSITION_NED':
                self.x, self.y, self.z = msg.x, msg.y, msg.z
            elif msg_type == 'ATTITUDE':
                self.yaw = msg.yaw 
            elif msg_type == 'GLOBAL_POSITION_INT':
                self.alt = msg.relative_alt / 1000.0

def send_global_velocity(master, vx, vy, vz=0, yaw=0):
    type_mask = int(0b0000101111000111) 
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
            0, 0, 0, vx, vy, vz, 0, 0, 0, yaw, 0)
    )

def arm_and_takeoff(master, telem, target_altitude):
    print("Switching to GUIDED mode...")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4) 
    time.sleep(1)
    print("Arming motors...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    time.sleep(2)
    print(f"Forced climb to {target_altitude} meters...")
    
    while True:
        telem.update(master)
        send_global_velocity(master, vx=0, vy=0, vz=-1.5) 
        if telem.alt >= target_altitude * 0.90: 
            send_global_velocity(master, vx=0, vy=0, vz=0)
            time.sleep(1)
            break
        time.sleep(0.2) 

def generate_grid_4x4(x_min, x_max, y_min, y_max):
    waypoints = []
    x_points = np.linspace(x_min, x_max, 4)
    y_points = np.linspace(y_min, y_max, 4)

    for i, y in enumerate(y_points):
        if i % 2 == 0:  
            for x in x_points:
                waypoints.append((x, y))
        else:           
            for x in reversed(x_points):
                waypoints.append((x, y))
                
    return waypoints

def process_vision(frame):
    H, W, _ = frame.shape
    col_w = W // 3          
    
    v_state = "CLEAR" 

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
    mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 5000: 
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            
            if (x + w) > col_w:
                v_state = "EVADE"   
            else:
                v_state = "TRACE"   

    cv2.line(frame, (col_w, 0), (col_w, H), (255, 255, 255), 1)
    cv2.line(frame, (2 * col_w, 0), (2 * col_w, H), (255, 255, 255), 1)

    cv2.putText(frame, f"STATE: {v_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.imshow("Downward Camera - Vision Core", frame)
    cv2.waitKey(1) 

    return v_state

def main():
    print("Connecting to ArduPilot...")
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    master.wait_heartbeat()
    
    viewer = CameraViewer()
    
    print("Waiting for camera feed...")
    while viewer.latest_frame is None:
        time.sleep(0.1)
    print("Camera feed established!")

    telem = Telemetry()
    arm_and_takeoff(master, telem, 10.0)

    waypoints = generate_grid_4x4(X_MIN, X_MAX, Y_MIN, Y_MAX)
    
    wp_start_x, wp_start_y = waypoints[0]
    while True:
        telem.update(master)
        dist = math.sqrt((wp_start_x - telem.x)**2 + (wp_start_y - telem.y)**2)
        if dist < TOLERANCE: break
        target_yaw = math.atan2(wp_start_y - telem.y, wp_start_x - telem.x)
        vx = ((wp_start_x - telem.x) / dist) * MAX_SPEED
        vy = ((wp_start_y - telem.y) / dist) * MAX_SPEED
        send_global_velocity(master, vx, vy, 0, yaw=target_yaw)
        time.sleep(0.1)

    state = "SWEEP"
    target_index = 0
    hover_start_time = 0.0

    while target_index < len(waypoints):
        wp_x, wp_y = waypoints[target_index]
        
        print(f"\n--- Heading to WP {target_index+1}/16: X={wp_x:.1f}, Y={wp_y:.1f} ---")
        while True:
            telem.update(master)
            target_yaw = math.atan2(wp_y - telem.y, wp_x - telem.x)
            yaw_error = (target_yaw - telem.yaw + math.pi) % (2 * math.pi) - math.pi
            
            if abs(yaw_error) < 0.15: break
            send_global_velocity(master, 0, 0, 0, yaw=target_yaw)
            time.sleep(0.1)

        while True:
            frame = viewer.latest_frame.copy()
            vision_state = process_vision(frame)
            telem.update(master)

            if state == "SWEEP":
                if vision_state == "EVADE" or vision_state == "TRACE":
                    print("Red Zone detected! Halting sweep. Initiating right-evasion.")
                    state = "EVADING"
                else:
                    dist = math.sqrt((wp_x - telem.x)**2 + (wp_y - telem.y)**2)
                    if dist < TOLERANCE: 
                        print(f"Passed WP {target_index+1} successfully.")
                        target_index += 1  
                        break 
                    
                    vx = ((wp_x - telem.x) / dist) * MAX_SPEED
                    vy = ((wp_y - telem.y) / dist) * MAX_SPEED
                    send_global_velocity(master, vx, vy, 0, yaw=target_yaw)

            elif state == "EVADING":
                vx = -EVADE_SPEED * math.sin(target_yaw)
                vy = EVADE_SPEED * math.cos(target_yaw)
                send_global_velocity(master, vx, vy, 0, yaw=target_yaw) 
                
                if vision_state == "TRACE":
                    print("Red safely in left column. Moving forward blind.")
                    state = "TRACING"
                elif vision_state == "CLEAR":
                    print("Red completely vanished during lateral shift! Moving straight to hover.")
                    hover_start_time = time.time()
                    state = "HOVER_PAUSE"

            elif state == "TRACING":
                vx = TRACE_SPEED * math.cos(target_yaw)
                vy = TRACE_SPEED * math.sin(target_yaw)
                send_global_velocity(master, vx, vy, 0, yaw=target_yaw) 
                
                if vision_state == "CLEAR":
                    print("Red zone completely cleared FOV. Hovering in place for 3 seconds...")
                    hover_start_time = time.time()
                    state = "HOVER_PAUSE"
                elif vision_state == "EVADE": 
                    state = "EVADING"

            elif state == "HOVER_PAUSE":
                send_global_velocity(master, 0, 0, 0, yaw=target_yaw)
                
                if time.time() - hover_start_time >= 3.0:
                    print("Hover complete. Finding next sequential waypoint...")
                    state = "FIND_NEXT"

            elif state == "FIND_NEXT":
                # STRICT SEQUENTIAL ORDER CHECK
                while target_index < len(waypoints):
                    wp = waypoints[target_index]
                    dx = wp[0] - telem.x
                    dy = wp[1] - telem.y
                    
                    # Project the distance onto our forward heading
                    forward_dist = dx * math.cos(target_yaw) + dy * math.sin(target_yaw)
                    
                    # If forward_dist is positive, the waypoint is IN FRONT of us.
                    if forward_dist > 0.5: 
                        break # Found the next valid sequential waypoint!
                    else:
                        print(f"Skipping WP {target_index+1} (we already passed it while evading).")
                        target_index += 1
                
                if target_index >= len(waypoints):
                    print("Mission complete or exhausted all waypoints!")
                    break
                            
                recovery_x, recovery_y = waypoints[target_index]
                print(f"-> Selected safe coordinate: WP {target_index+1}. Translating...")
                state = "RECOVERING"

            elif state == "RECOVERING":
                if vision_state == "EVADE" or vision_state == "TRACE":
                    print("Red Zone re-detected during recovery! Canceling translation and dodging.")
                    state = "EVADING"
                else:
                    dx = recovery_x - telem.x
                    dy = recovery_y - telem.y
                    dist = math.sqrt(dx**2 + dy**2)
                    
                    if dist < TOLERANCE:
                        print(f"Arrived at safe WP {target_index+1}. Resuming sequential Boustrophedon flow.")
                        target_index += 1  
                        state = "SWEEP"
                        break 
                    else:
                        vx = (dx / dist) * MAX_SPEED
                        vy = (dy / dist) * MAX_SPEED
                        send_global_velocity(master, vx, vy, 0, yaw=target_yaw)

            time.sleep(0.05) 

    print("\n--- 16-POINT MATRIX SWEEP COMPLETE ---")
    send_global_velocity(master, 0, 0, 0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
