import os
import sys
import time
import datetime
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from config import Config  # Import your config file

config = Config.data_collection
class ConfigMain:
    # Example configuration parameters
    vehicle_name = config['vehicle_name']  # or 'rover', adjust as needed
    path_to_e2e_data = config['path_to_e2e_data'] #   default path to end-to-end data
    vehicle_control_topic = config['vehicle_control_topic'] 
    base_pose_topic = config['base_pose_topic'] 
    camera_image_topic = config['camera_image_topic'] 
    image_crop = config['image_crop_x1']  # whether to crop images
    image_crop_x1 = config['image_crop_x1']
    image_crop_y1 = config['image_crop_y1']
    image_crop_x2 = config['image_crop_x2']
    image_crop_y2 = config['image_crop_y2']

    image_width = config['image_width']
    image_height = config['image_height']
    version = config['version'] # dataset version

 

# You can add more configuration parameters as needed
class DataCollection(Node):
    def __init__(self):
        super().__init__('data_collection')

        self.steering = 0
        self.throttle = 0
        self.brake = 0

        self.vel_x = self.vel_y = self.vel_z = 0
        self.vel = 0
        self.pos_x = self.pos_y = self.pos_z = 0

        self.bridge = CvBridge()

        self.vehicle_name = ConfigMain.vehicle_name
        if  self.vehicle_name == 'fusion':
            from fusion.msg import Control
        elif  self.vehicle_name == 'mrover':
            from mrover.msg import Control
        else:
            exit(config['vehicle_name'] + 'not supported vehicle.')
        self.vehicle_control_topic = ConfigMain.vehicle_control_topic
        self.base_pose_topic = ConfigMain.base_pose_topic
        self.camera_image_topic = ConfigMain.camera_image_topic
        self.image_crop = ConfigMain.image_crop
        self.image_crop_x1 = ConfigMain.image_crop_x1
        self.image_crop_x2 = ConfigMain.image_crop_x2
        self.image_crop_y1 = ConfigMain.image_crop_y1
        self.image_crop_y2 = ConfigMain.image_crop_y2
        self.version = ConfigMain.version

        data_id = sys.argv[1] if len(sys.argv) > 1 else 'default_data_id'
        path_to_e2e_data = self.declare_parameter('path_to_e2e_data', ConfigMain.path_to_e2e_data).value
        path = os.path.join(path_to_e2e_data, data_id, datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))

        if not os.path.exists(path):
            os.makedirs(path)
            self.get_logger().info(f'New folder created: {path}')

        self.text = open(os.path.join(path, f'{datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}.txt'), 'w+')
        self.path = path

        self.create_subscription(Control, self.vehicle_control_topic, self.steering_throttle_cb, 10)
        self.create_subscription(Odometry, self.base_pose_topic, self.pos_vel_cb, 10)
        self.create_subscription(Image, self.camera_image_topic, self.recorder_cb, 10)

    def calc_velocity(self, x, y, z):
        return math.sqrt(x ** 2 + y ** 2 + z ** 2)

    def steering_throttle_cb(self, msg):
        self.throttle = msg.throttle
        self.steering = msg.angular.z
        self.brake = msg.brake

    def pos_vel_cb(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z

        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z
        self.vel = self.calc_velocity(self.vel_x, self.vel_y, self.vel_z)

    def recorder_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        cropped = None
        if self.image_crop:
            cropped = img[self.image_crop_y1:self.image_crop_y2, self.image_crop_x1:self.image_crop_x2]

        time_stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")
        file_full_path = os.path.join(self.path, f'{time_stamp}.jpg')

        if self.image_crop:
            cv2.imwrite(file_full_path, cropped)
        else:
            cv2.imwrite(file_full_path, img)

        self.get_logger().info(f'Saved image: {file_full_path}')

        unix_time = time.time()

        if self.version >= 0.92:
            line = f"{time_stamp}.jpg,{self.steering},{self.throttle},{self.brake},{unix_time},{self.vel},{self.vel_x},{self.vel_y},{self.vel_z},{self.pos_x},{self.pos_y},{self.pos_z}\n"
        else:
            line = f"{time_stamp}.jpg,{self.steering},{self.throttle},{unix_time},{self.vel},{self.vel_x},{self.vel_y},{self.vel_z},{self.pos_x},{self.pos_y},{self.pos_z}\n"

        self.text.write(line)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print('Usage: ')
        exit('$ ros2 run package_name executable_name your_data_id')

    dc = DataCollection()

    try:
        rclpy.spin(dc)
    except KeyboardInterrupt:
        pass
    finally:
        dc.destroy_node()
        rclpy.shutdown()
        print("\nBye...")

if __name__ == '__main__':
    main()
