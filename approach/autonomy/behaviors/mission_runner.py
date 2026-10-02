import cv2
import numpy as np
import sys
import os
import time
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# Import your custom detectors
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from autonomy.perception.hybrid_banner_detector import HybridBannerDetector
from autonomy.perception.qr_detector import QRDetector

frame_down = None
frame_forward = None
current_alt = 0.0

def on_image_down(msg):
    global frame_down
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    frame_down = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def on_image_forward(msg):
    global frame_forward
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    frame_forward = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def send_velocity_cmd(master, vx, vy, vz):
    type_mask = int(0b0000111111000111)
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, type_mask,
            0, 0, 0, 
            vx, vy, vz, 
            0, 0, 0, 0, 0)
    )

def main():
    global frame_down, frame_forward, current_alt
    
    print("Connecting to ArduPilot...")
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14552")
    master.wait_heartbeat()
    print("Heartbeat received!")

    qr_detector = QRDetector()
    banner_detector = HybridBannerDetector()

    node = Node()
    node.subscribe(Image, "/iris/camera_downward/image_raw", on_image_down)
    node.subscribe(Image, "/iris/camera_forward/image_raw", on_image_forward)

    state = "TAKEOFF"
    state_start_time = time.time()
    centered_frames_count = 0
    
    print("Mission Ready: The script is waiting. Go to MAVProxy and type: takeoff 5")

    try:
        while True:
            if frame_down is None or frame_forward is None:
                print(f"Waiting for video feeds... Downward: {frame_down is not None} | Forward: {frame_forward is not None}")
                time.sleep(1)
                continue

            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                current_alt = msg.relative_alt / 1000.0

            down_res = qr_detector.detect(frame_down)
            fwd_res = banner_detector.detect(frame_forward)

            # ==========================================
            # STATE MACHINE LOGIC
            # ==========================================
            
            if state == "TAKEOFF":
                if current_alt > 4.8:
                    print("Altitude reached! Python is taking over. Moving 1m forward.")
                    state = "FORWARD"
                    state_start_time = time.time()

            elif state == "FORWARD":
                send_velocity_cmd(master, vx=0.5, vy=0, vz=0)
                if time.time() - state_start_time > 2.0:
                    print("Moved 1m forward. Initiating QR Centering.")
                    send_velocity_cmd(master, vx=0, vy=0, vz=0)
                    state = "QR_CENTERING"
                    centered_frames_count = 0

            elif state == "QR_CENTERING":
                if down_res["detected"]:
                    vx = max(-0.5, min(0.5, -down_res["error_y"] * 0.003))
                    vy = max(-0.5, min(0.5, down_res["error_x"] * 0.003))
                    send_velocity_cmd(master, vx, vy, vz=0)

                    if abs(down_res["error_x"]) < 20 and abs(down_res["error_y"]) < 20:
                        centered_frames_count += 1
                    else:
                        centered_frames_count = 0
                        
                    if centered_frames_count > 30:
                        print("QR Centered! Snapping picture...")
                        
                        save_dir = os.path.expanduser("~/sae_mission2/approach/qr_captures")
                        os.makedirs(save_dir, exist_ok=True)
                        timestamp = int(time.time())
                        filename = os.path.join(save_dir, f"qr_lock_{timestamp}.jpg")
                        cv2.imwrite(filename, frame_down)
                        print(f"Picture saved successfully to: {filename}")
                        
                        print("Moving 2m backward...")
                        state = "MOVE_BACK"
                        state_start_time = time.time()
                else:
                    send_velocity_cmd(master, 0, 0, 0)

            elif state == "MOVE_BACK":
                send_velocity_cmd(master, vx=-0.5, vy=0, vz=0)
                # Changed timer to 4.0 seconds for 2 meters of travel
                if time.time() - state_start_time > 4.0:
                    print("Moved 2m backward. Moving 2m down...")
                    send_velocity_cmd(master, vx=0, vy=0, vz=0)
                    state = "MOVE_DOWN"
                    state_start_time = time.time()

            elif state == "MOVE_DOWN":
                send_velocity_cmd(master, vx=0, vy=0, vz=0.5)
                if time.time() - state_start_time > 4.0:
                    print("Moved 2m down. Searching left for banner...")
                    send_velocity_cmd(master, vx=0, vy=0, vz=0)
                    state = "SEARCH_BANNER"

            elif state == "SEARCH_BANNER":
                if not fwd_res["detected"]:
                    send_velocity_cmd(master, vx=0, vy=-0.5, vz=0)
                else:
                    print("Banner Detected! Initiating Banner Centering.")
                    send_velocity_cmd(master, vx=0, vy=0, vz=0)
                    state = "CENTER_BANNER"
                    centered_frames_count = 0

            elif state == "CENTER_BANNER":
                if fwd_res["detected"]:
                    vy = max(-0.5, min(0.5, fwd_res["error_x"] * 0.003))
                    vz = max(-0.5, min(0.5, fwd_res["error_y"] * 0.003))
                    send_velocity_cmd(master, vx=0, vy=vy, vz=vz)
                    
                    if abs(fwd_res["error_x"]) < 20 and abs(fwd_res["error_y"]) < 20:
                        centered_frames_count += 1
                    else:
                        centered_frames_count = 0
                        
                    if centered_frames_count > 30:
                        print("Banner Centered! Holding position.")
                        state = "HOVER"
                else:
                    send_velocity_cmd(master, 0, 0, 0)

            elif state == "HOVER":
                send_velocity_cmd(master, vx=0, vy=0, vz=0)
                cv2.putText(fwd_res["debug_frame"], "MISSION COMPLETE", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

            # ==========================================
            # DISPLAY WINDOWS
            # ==========================================
            cv2.putText(down_res["debug_frame"], f"ALT: {current_alt:.1f}m | STATE: {state}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(fwd_res["debug_frame"], f"ALT: {current_alt:.1f}m | STATE: {state}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("Downward Camera", down_res["debug_frame"])
            cv2.imshow("Forward Camera", fwd_res["debug_frame"])
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
                
    except KeyboardInterrupt:
        pass
        
    send_velocity_cmd(master, 0, 0, 0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
