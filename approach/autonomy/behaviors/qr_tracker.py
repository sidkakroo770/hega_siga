import cv2
import numpy as np
import sys
import os
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# Import the QR Detector
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from autonomy.perception.qr_detector import QRDetector

latest_frame = None

def on_image(msg):
    global latest_frame
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def send_velocity_cmd(master, vx, vy):
    """
    Commands planar velocity in meters per second.
    vx: forward (positive) / backward (negative)
    vy: right (positive) / left (negative)
    """
    type_mask = int(0b0000111111000111) # Ignore position and acceleration
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, type_mask,
            0, 0, 0,    
            vx, vy, 0,  
            0, 0, 0,    
            0, 0)       
    )

def main():
    global latest_frame
    
    print("Connecting to ArduPilot via MAVLink...")
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    master.wait_heartbeat()
    print("Heartbeat received! Ready for Downward QR Centering.")

    detector = QRDetector()
    node = Node()
    
    # Updated to the correct Iris downward camera topic!
    topic = "/iris/camera_downward/image_raw"
    node.subscribe(Image, topic, on_image)

    # Controller tuning
    Kp_x = 0.003  # Aggressiveness for forward/backward
    Kp_y = 0.003  # Aggressiveness for left/right
    deadzone = 20 # Pixel tolerance to prevent micro-stutters

    try:
        while True:
            if latest_frame is not None:
                frame = latest_frame.copy()
                result = detector.detect(frame)
                
                if result["detected"]:
                    error_x = result["error_x"]
                    error_y = result["error_y"]
                    
                    vx = 0.0
                    vy = 0.0
                    
                    # Forward/Backward Correction (Invert error_y for downward cam physics)
                    if abs(error_y) > deadzone:
                        vx = max(-1.0, min(1.0, -error_y * Kp_x))
                        
                    # Left/Right Correction
                    if abs(error_x) > deadzone:
                        vy = max(-1.0, min(1.0, error_x * Kp_y))
                        
                    if vx != 0.0 or vy != 0.0:
                        send_velocity_cmd(master, vx, vy)
                        cv2.putText(result["debug_frame"], f"CENTERING | vx: {vx:.2f}, vy: {vy:.2f}", 
                                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    else:
                        send_velocity_cmd(master, 0, 0)
                        cv2.putText(result["debug_frame"], "QR CENTERED", 
                                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                else:
                    # Hover in place if QR code is not visible
                    send_velocity_cmd(master, 0, 0)

                cv2.imshow("Downward QR Tracking", result["debug_frame"])
                
            if cv2.waitKey(30) & 0xFF == 27: # Press ESC to stop
                break
                
    except KeyboardInterrupt:
        pass
        
    send_velocity_cmd(master, 0, 0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
