import sys, subprocess, time
import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

class CameraViewer:
    def __init__(self):
        self.node = Node()
        topic = self.find_camera_topic()
        print(f"[CAM1] Subscribing to active image topic: {topic}")
        self.node.subscribe(Image, topic, self.image_callback)

    def find_camera_topic(self):
        return "/iris/camera_downward/image_raw" 
    def image_callback(self, msg: Image):
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        
        if msg.pixel_format_type == 3: # RGB_INT8
            frame = img_data.reshape((msg.height, msg.width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.pixel_format_type == 1: # Monochrome
            frame = img_data.reshape((msg.height, msg.width))
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            frame = img_data.reshape((msg.height, msg.width, -1))
        
        cv2.imshow("Downward Camera (CAM1)", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    print("[CAM1] Starting Zero-Latency Camera Listener...")
    viewer = CameraViewer()
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
