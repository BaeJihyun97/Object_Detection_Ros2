# display_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.subscription_results = self.create_subscription(Detection2DArray, 'camera/processed_image', self.results_callback, 10)
        self.subscription_images = self.create_subscription(Image, 'camera/image_raw', self.images_callback, 10)
        self.bridge = CvBridge()
        self.image = None
        self.detections = None
        self.count = 0 # test

    def images_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # msg -> cv2
            self.image = image
            self.display_image()
        except CvBridgeError as e:
            self.get_logger().error(e)

        


    def results_callback(self, msg):
        self.detections = msg
        self.display_image()

    def display_image(self):
        # if self.image is not None:
        #     display_image = self.image.copy()
            
        #     cv2.imshow('Detected Objects', display_image)
        #     cv2.waitKey(1)

        if self.image is not None and self.detections is not None:
            # test message
            self.count += 1
            if self.count % 50 == 0:
                print("displayed ", self.count)

            display_image = self.image.copy()

            detections = self.detections.detections # array of Detection2D

            for detection in detections:
                box = detection.bbox
                label = detection.results[0].hypothesis.class_id
                prob = detection.results[0].hypothesis.score
                x, y, w, h = box.center.position.x, box.center.position.y, box.size_x, box.size_y
                x1, y1, x2, y2 = int(x - w / 2), int(y - h / 2), int(x + w / 2), int(y + h / 2)
                cv2.rectangle(display_image, (x1, y1), (x2, y2), (255, 255, 0), 4)
                cv2.putText(display_image, f"{label}: {prob}", 
                            (x1+20, y1+40),             # location
                            cv2.FONT_HERSHEY_SIMPLEX,   # font
                            1,                          # font scale
                            (255, 0, 255), 2)           # line type
            
            cv2.imshow('Detected Objects', display_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()