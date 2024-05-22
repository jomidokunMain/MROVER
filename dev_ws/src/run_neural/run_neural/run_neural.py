import threading
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math
import numpy as np

import sys
import os

import const
from image_converter import ImageConverter
from drive_run import DriveRun
from config import Config
from image_process import ImageProcess

if Config.data_collection['vehicle_name'] == 'fusion':
    from fusion.msg import Control
elif Config.data_collection['vehicle_name'] == 'rover':
    from geometry_msgs.msg import Twist
    from rover.msg import Control
else:
    exit(Config.data_collection['vehicle_name'] + ' not supported vehicle.')

config = Config.neural_net
velocity = 0.0

class NeuralControl(Node):
    def __init__(self, weight_file_name):
        super().__init__('run_neural')
        self.ic = ImageConverter()
        self.image_process = ImageProcess()
        self.rate = self.create_rate(30)
        self.drive = DriveRun(weight_file_name)
        self.image_sub = self.create_subscription(Image, Config.data_collection['camera_image_topic'], self._controller_cb, QoSProfile(depth=10))
        self.image = None
        self.image_processed = False
        self.braking = False

    def _controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)
        cropped = img[Config.data_collection['image_crop_y1']:Config.data_collection['image_crop_y2'],
                      Config.data_collection['image_crop_x1']:Config.data_collection['image_crop_x2']]
                      
        img = cv2.resize(cropped, (config['input_image_width'],
                                   config['input_image_height']))
                                  
        self.image = self.image_process.process(img)

        if config['lstm']:
            self.image = np.array(self.image).reshape(1, 
                                 config['input_image_height'],
                                 config['input_image_width'],
                                 config['input_image_depth'])
        self.image_processed = True
        
    def _timer_cb(self):
        self.braking = False

    def apply_brake(self):
        self.braking = True
        timer = threading.Timer(Config.run_neural['brake_apply_sec'], self._timer_cb) 
        timer.start()

def pos_vel_cb(value):
    global velocity

    vel_x = value.twist.twist.linear.x 
    vel_y = value.twist.twist.linear.y
    vel_z = value.twist.twist.linear.z
    
    velocity = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        exit('Usage:\n$ ros2 run run_neural run_neural.py weight_file_name')

    neural_control = NeuralControl(sys.argv[1])
    
    pos_vel_sub = neural_control.create_subscription(Odometry, Config.data_collection['base_pose_topic'], pos_vel_cb, QoSProfile(depth=10))
    joy_pub = neural_control.create_publisher(Control, Config.data_collection['vehicle_control_topic'], 10)
    joy_data = Control()

    if Config.data_collection['vehicle_name'] == 'rover':
        joy_pub4mavros = neural_control.create_publisher(Twist, Config.data_collection['mavros_cmd_vel_topic'], 20)

    print('\nStart running. Vroom. Vroom. Vroooooom......')
    print('steer \tthrt: \tbrake \tvelocity')

    use_predicted_throttle = True if config['num_outputs'] == 2 else False
    while rclpy.ok():

        if not neural_control.image_processed:
            rclpy.spin_once(neural_control)
            continue
        
        if config['num_inputs'] == 2:
            prediction = neural_control.drive.run((neural_control.image, velocity))
            if config['num_outputs'] == 2:
                joy_data.steer = prediction[0][0]
                joy_data.throttle = prediction[0][1]
            else:
                joy_data.steer = prediction[0][0]
        else:
            prediction = neural_control.drive.run((neural_control.image,))
            if config['num_outputs'] == 2:
                joy_data.steer = prediction[0][0]
                joy_data.throttle = prediction[0][1]
            else:
                joy_data.steer = prediction[0][0]
        
        is_sharp_turn = False

        if not neural_control.braking: 
            if velocity < Config.run_neural['velocity_0']:
                joy_data.throttle = Config.run_neural['throttle_default']
                joy_data.brake = 0
            elif abs(joy_data.steer) > Config.run_neural['sharp_turn_min']:
                is_sharp_turn = True
            
            if is_sharp_turn or velocity > Config.run_neural['max_vel']: 
                joy_data.throttle = Config.run_neural['throttle_sharp_turn']
                joy_data.brake = Config.run_neural['brake_val']
                neural_control.apply_brake()
            else:
                if not use_predicted_throttle:
                    joy_data.throttle = Config.run_neural['throttle_default']
                joy_data.brake = 0

        joy_pub.publish(joy_data)

        if Config.data_collection['vehicle_name'] == 'rover':
            joy_data4mavros = Twist()
            if neural_control.braking:
                joy_data4mavros.linear.x = 0
                joy_data4mavros.linear.y = 0
            else: 
                joy_data4mavros.linear.x = joy_data.throttle * Config.run_neural['scale_factor_throttle']
                joy_data4mavros.linear.y = joy_data.steer * Config.run_neural['scale_factor_steering']

            joy_pub4mavros.publish(joy_data4mavros)

        cur_output = '{0:.3f} \t{1:.3f} \t{2:.3f} \t{3:.3f}\r'.format(joy_data.steer, 
                          joy_data.throttle, joy_data.brake, velocity)

        sys.stdout.write(cur_output)
        sys.stdout.flush()
        
        neural_control.image_processed = False
        neural_control.rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('\nShutdown requested. Exiting...')
        rclpy.shutdown()
