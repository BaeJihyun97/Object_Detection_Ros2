# ip_webcam_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class IPCameraPublisher(Node):
    def __init__(self):
        super().__init__('ip_camera_publisher')
        
        # parameters. 다른 방법으론 ros2의 launch 기능으로 parameters 관리 가능
        # 파일 경로는 setup.py 의 data_files = [('share/' + package_name + '/conf', glob('conf/*.yaml'))] 부분
        # build 없이 바꾸려면 share/pakage_name/conf 디렉토리의 conf.yaml 파일을 바꾸면 됨.
        config_file_path = os.path.join(get_package_share_directory('ip_camera_test'), 'conf', 'conf.yaml')
        with open(config_file_path, 'r') as file:
            self.conf = yaml.safe_load(file)

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(f"http://{self.conf['WEBCAM_IP']}:8080/video")
        self.count = 0

        # buffer size
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    def timer_callback(self):
        for _ in range(self.conf['RETRY']):
            ret, frame = self.cap.read()
            if ret:
                # test message
                self.count += 1
                if self.count % 100 == 0:
                    print("webcam ", self.count)

                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.publisher_.publish(msg)
                break
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = IPCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()