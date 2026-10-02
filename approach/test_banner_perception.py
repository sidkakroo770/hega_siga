import cv2
import numpy as np
import time
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from autonomy.perception.orb_banner_detector import ORBBannerDetector
# Global variable to store the newest frame
latest_frame = None

def on_image(msg):
    global latest_frame
    # Convert Gazebo Image message to OpenCV format
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((msg.height, msg.width, 3))
    # Gazebo sends RGB, OpenCV expects BGR
    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def main():
    global latest_frame
    detector = ORBBannerDetector()
    node = Node()
    topic = "/iris/camera_forward/image_raw"
    
    # Subscribe to CAM2
    if node.subscribe(Image, topic, on_image):
        print(f"Subscribed to {topic}.")
        print("Opening Unified CAM2 Viewer... Press ESC to close.")
    else:
        print(f"Failed to subscribe to {topic}")
        return
        
    try:
        # Main thread loop: this keeps OpenCV happy and running smoothly
        while True:
            if latest_frame is not None:
                # 1. Grab a copy of the latest frame to avoid overwriting during detection
                frame_to_process = latest_frame.copy()
                
                # 2. Run our perception module
                result = detector.detect(frame_to_process)
                
                # 3. Show the output (CAM2 feed with the detection boxes drawn on it)
                cv2.imshow("CAM2 + Green Banner Detection", result["debug_frame"])
                
            # OpenCV waitKey MUST be in the main thread. 
            # 30ms gives us roughly ~30 FPS UI refresh rate.
            if cv2.waitKey(30) & 0xFF == 27: # 27 is the ESC key
                break
                
    except KeyboardInterrupt:
        pass
        
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
