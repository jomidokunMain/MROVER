#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 13:23:14 2017
History:
11/28/2020: modified for OSCAR 

@author: jaerock
"""

import threading 
import cv2
import time
import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
import math
from rover.msg import Control
import sys
import os

import const
from image_converter import ImageConverter
from drive_run import DriveRun
from config import Config
from image_process import ImageProcess
import gpu_options

config = Config.neural_net
velocity = 0

class NeuralControl:
    def __init__(self, weight_file_name):
        
        self.ic = ImageConverter()
        self.image_process = ImageProcess()
        self.rate = rospy.Rate(30)
        self.drive= DriveRun(weight_file_name)
        rospy.Subscriber(Config.data_collection['camera_image_topic'], Image, self._controller_cb)
        self.image = None
        self.image_processed = False
        #self.config = Config()
        self.braking = False


    def _controller_cb(self, image): 
        img = self.ic.imgmsg_to_opencv(image)
        cropped = img[Config.data_collection['image_crop_y1']:Config.data_collection['image_crop_y2'],
                      Config.data_collection['image_crop_x1']:Config.data_collection['image_crop_x2']]
                      
        img = cv2.resize(cropped, (config['input_image_width'],
                                   config['input_image_height']))
                                  
        self.image = self.image_process.process(img)

        ## this is for CNN-LSTM net models
        if config['lstm'] is True:
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

      
def main(weight_file_name):
    gpu_options.set()
    # ready for neural network
    neural_control = NeuralControl(weight_file_name)
    
    rospy.Subscriber(Config.data_collection['base_pose_topic'], Odometry, pos_vel_cb)
    # ready for /bolt topic publisher
    joy_pub = rospy.Publisher(Config.data_collection['vehicle_control_topic'], Control, queue_size = 20)
    joy_data = Control()
    r = rospy.Rate(30)

    joy_pub4mavros2 = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)
    joy_data4mavros2 = AttitudeTarget()
    joy_data.throttle= Config.run_neural['throttle_default']
    #joy_pub4mavros2.publish(joy_data4mavros2)


    print('\nStart running. Vroom. Vroom. Vroooooom......')
    print('steer \tthrt: \tbrake \tvelocity')

    use_predicted_throttle = True if config['num_outputs'] == 2 else False
    while not rospy.is_shutdown():
        if neural_control.image_processed is False:
            continue
        
        # predicted steering angle from an input image
        if config['num_inputs'] == 2:
            prediction = neural_control.drive.run((neural_control.image, velocity))
            if config['num_outputs'] == 2:
                # prediction is [ [] ] numpy.ndarray
                joy_data.steer = prediction[0][0]
                joy_data.throttle = prediction[0][1]
            else: # num_outputs is 1
                joy_data.steer = prediction[0][0]
        else: # num_inputs is 1
            prediction = neural_control.drive.run((neural_control.image, ))
            if config['num_outputs'] == 2:
                # prediction is [ [] ] numpy.ndarray
                joy_data.steer = prediction[0][0]
                joy_data.throttle = prediction[0][1]
            else: # num_outputs is 1
                joy_data.steer = prediction[0][0]
            
        #############################
        ## very very simple controller
        ## 

        '''is_sharp_turn = False
        # if brake is not already applied and sharp turn
        if neural_control.braking is False: 
            if velocity < Config.run_neural['velocity_0']: # too slow then no braking
                joy_data.throttle = Config.run_neural['throttle_default'] # apply default throttle
                joy_data4mavros.linear.y = joy_data.throttle # apply default throttle
                joy_data.brake = 0
            elif abs(joy_data.steer) > Config.run_neural['sharp_turn_min']:
                is_sharp    joy_pub4mavros2 = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)
    joy_data4mavros2 = AttitudeTarget()
    joy_data4mavros2.thrust=0.5
    joy_pub4mavros2.publish(joy_data4mavros2)
    r = rospy.Rate(30)_turn = True
            
            if is_sharp_turn or velocity > Config.run_neural['max_vel']: 
                joy_data.throttle = Config.run_neural['throttle_sharp_turn']
                joy_data4mavros.linear.y = joy_data.throttle
                joy_data.brake = Config.run_neural['brake_val']
                neural_control.apply_brake()
            else:
                if use_predicted_throttle is False:
                    joy_data.throttle = Config.run_neural['throttle_default']
                    joy_data4mavros.linear.y = joy_data.throttle
                joy_data.brake = 0pos_vel_cb
                joy_data4mavros.linear.x = joy_data.steer
                local_target_pub.publish()'''


        
        ##############################    
        ## publish mavros control topic
        

        joy_data4mavros2.thrust = Config.run_neural['throttle_default']
        if joy_data.steer >= 0:
        	if 0 < joy_data.steer <= 0.01:
        		joy_data4mavros2.orientation.x =0
        		joy_data4mavros2.orientation.y =0

        	else:
        		joy_data4mavros2.orientation.x = joy_data.steer
        		joy_data4mavros2.orientation.y = 0

        else:
        	if -0.01 < joy_data.steer < 0:
        		joy_data4mavros2.orientation.x =0
        		joy_data4mavros2.orientation.y =0

        	else:
        		joy_data4mavros2.orientation.x = 0
        		joy_data4mavros2.orientation.y = joy_data.steer
         	#if joy_data.steer <= -0.11:
        	#joy_data4mavros2.orientation.y =joy_data.steer
        	#joy_data4mavros2.orientation.x =0
        	#else:
        		#joy_data4mavros2.orientation.y = 0    


        joy_pub.publish(joy_data)
        joy_pub4mavros2.publish(joy_data4mavros2)

        #if Config.data_collection['vehicle_name'] == 'rover':
            #joy_data4mavros = Twist()
            #if neural_control.braking is True:
                #joy_data4mavros.linear.x = 0
                #joy_data4mavros.linear.y = 0
            #else: 
                #joy_data4mavros.linear.x = joy_data.throttle*Config.run_neural['scale_factor_steering']
                #joy_data4mavros.linear.y = joy_data.steer*Config.run_neural['scale_factor_throttle']

            #joy_pub4mavros.publish(joy_data4mavros)



        ## print out
        cur_output = '{0:.3f} \t{1:.3f} \t{2:.3f} \t{3:.3f}\r'.format(joy_data.steer, 
                          joy_data4mavros2.thrust, joy_data.brake, velocity)

        sys.stdout.write(cur_output)
        sys.stdout.flush()
   
        ## ready for processing a new input image
        neural_control.image_processed = False
        neural_control.rate.sleep()
        r.sleep() 




if __name__ == "__main__":
    	
    rospy.init_node('run_neural')
    '''joy_pub4mavros2 = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)
    joy_data4mavros2 = AttitudeTarget()
    joy_data4mavros2.thrust=0.5
    joy_pub4mavros2.publish(joy_data4mavros2)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
    	joy_pub4mavros2.publish(joy_data4mavros2)'''

    try:
        nc = NeuralControl 
        #NeuralControl()rospy.init_node('run_neural')
        if len(sys.argv) != 2:
            
            exit('Usage:\n$ rosrun run_neural run_neural5.py weight_file_name')
        main(sys.argv[1])



    #local_target_pub.publish()
    #rospy.spin()
    	#r.sleep()
    except KeyboardInterrupt:
        print ('\nShutdown requested. Exiting...')
        
