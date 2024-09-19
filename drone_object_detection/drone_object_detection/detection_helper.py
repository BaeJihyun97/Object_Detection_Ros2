from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import ObjectHypothesisWithPose, BoundingBox2D, Detection2D, Detection2DArray

from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory
import cv2
import os
import yaml

from drone_object_detection.utils import Timer
from drone_object_detection.model import Model

config_file_path = os.path.join(get_package_share_directory('drone_object_detection'), 'conf', 'conf.yaml')
with open(config_file_path, 'r') as file:
    conf = yaml.safe_load(file)

TOPIC_SUBSCRIPTION = conf['TOPIC_SUBSCRIPTION']
TOPIC_PUBLISHER = conf['TOPIC_PUBLISHER']
NET_TYPE = conf['NET_TYPE']


class DetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # subscriber : Image
        self.subscription = self.create_subscription(
            Image,
            TOPIC_SUBSCRIPTION,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # publisher : 2DARRAY
        self.publisher = self.create_publisher(Detection2DArray, TOPIC_PUBLISHER, 10)
        
        # model 
        self.predictor = Model(NET_TYPE)
        self.class_names = self.predictor.class_names

        # utils
        self.timer = Timer()

        # !!TEST!! TODO: delete for releasing
        self.timer.start()
        self.count = 0

    def listener_callback(self, msg):

        # convert msg image to cv2 image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # msg -> cv2
            display_image = image.copy()
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # bgr -> rgb
        except CvBridgeError as e:
            self.get_logger().error(e)

        # predict
        boxes, labels, probs = self.predictor.predict(image)
        


        # make publishing message
        detection_array = Detection2DArray()

        for box, label, prob in zip(boxes, labels, probs):
            x1, y1, x2, y2 = box
            '''
            https://github.com/ros-perception/vision_msgs (2024.07.23)
            vision_msgs 4.1.1

            ObjectHypothesis            = {string class_id, float64 score}
            ObjectHypothesisWithPose    = {ObjectHypothesis hypothesis, geometry_msgs/PoseWithCovariance pose}
            BoundingBox2D               = {vision_msgs/Pose2D center, float64 size_x, float64 size_y}
            Pose2D                      = {vision_msgs/Point2D position, float64 theta}
            Point2D                     = {float64 x, float64 y}
            Detection2D                 = {std_msgs/Header header, ObjectHypothesisWithPose[] results, BoundingBox2D bbox, string id}
            Detection2DArray            = {std_msgs/Header header, Detection2D[] detections} 
            std_msgs/Header             = {uint32 seq, time stamp, string frame_id}
            '''
            object_hypothesis_with_pose = ObjectHypothesisWithPose()
            object_hypothesis_with_pose.hypothesis.class_id = str(self.class_names[label])
            object_hypothesis_with_pose.hypothesis.score = float(prob)

            bounding_box = BoundingBox2D()
            bounding_box.center.position.x = float((x1 + x2)/2)
            bounding_box.center.position.y = float((y1 + y2)/2)
            bounding_box.center.theta = 0.0
            
            bounding_box.size_x = float(x2 - x1)
            bounding_box.size_y = float(y2 - y1)

            detection = Detection2D()
            detection.header = msg.header
            detection.results.append(object_hypothesis_with_pose)
            detection.bbox = bounding_box

            detection_array.header = msg.header
            detection_array.detections.append(detection)

            # label_str = str(self.class_names[label])

            # cv2.rectangle(display_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 0), 4)
            # cv2.putText(display_image, f"{label_str}: {float(prob)}", 
            #                 (int(x1), int(y1) - 10),             # location
            #                 cv2.FONT_HERSHEY_SIMPLEX,   # font
            #                 1,                          # font scale
            #                 (255, 0, 255), 2)           # line type

        # !!TEST!! TODO: delete for releasing
        self.count += 1
        if self.count % 50 == 0:
            interval = self.timer.end()
            print('detected 50/{:d} images Time: {:.2f}s.'.format(self.count, interval))
            print(msg.header.frame_id)  
            self.timer.start()

        # publishing message
        self.publisher.publish(detection_array)
        # display_image = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
        # self.publisher.publish(display_image)