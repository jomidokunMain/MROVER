#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from dynamic_reconfigure.client import Client

def imu_callback(msg):
    # Update pose_covariance_diagonal and twist_covariance_diagonal based on IMU data
    # Example: Let's say you want to set them based on the linear and angular accelerations
    pose_covariance_diagonal = [0.0, 0.0, 0.0, abs(msg.linear_acceleration.x), abs(msg.linear_acceleration.y), abs(msg.linear_acceleration.z)]
    twist_covariance_diagonal = [0.0, 0.0, 0.0, abs(msg.angular_velocity.x), abs(msg.angular_velocity.y), abs(msg.angular_velocity.z)]

    # Update the parameters using dynamic reconfigure
    client.update_configuration({"ackermann_steering_bot_controller/pose_covariance_diagonal": pose_covariance_diagonal,
                                  "ackermann_steering_bot_controller/twist_covariance_diagonal": twist_covariance_diagonal})

if __name__ == "__main__":
    rospy.init_node("imu_covariance_updater")

    # Create a dynamic reconfigure client to update parameters
    client = Client("ackermann_steering_bot_controller", timeout=30)

    # Subscribe to IMU data
    rospy.Subscriber("/imu_topic", Imu, imu_callback)

    rospy.spin()