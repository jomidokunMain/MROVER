#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import quaternion_from_euler

def cmd_vel_callback(msg):
    # Initialize the AttitudeTarget message
    attitude_msg = AttitudeTarget()
    
    # Map the /cmd_vel.linear.x to /mavros/setpoint_raw/attitude.thrust
    
    
     # Map thrust to linear.x
    if msg.linear.x > 0:
        attitude_msg.thrust = 0.6+0.5*msg.linear.x
        if attitude_msg.thrust == 0.5:
            attitude_msg.thrust =0
    elif msg.linear.x < 0:
        attitude_msg.thrust = 0.5-0.5*(-1*msg.linear.x)
        if attitude_msg.thrust == 0.5:
            attitude_msg.thrust = 0
        elif 0.2 <= attitude_msg.thrust < 0.5:
            attitude_msg.thrust = 0.25
        elif 0 < attitude_msg.thrust <= 0.02:
            attitude_msg.thrust =0.02
        elif msg.linear.x == -1:
            attitude_msg.thrust = 0.02
    else:
        attitude_msg.thrust = 0
    
    # Map the /cmd_vel.angular.z to /mavros/setpoint_raw/attitude.orientation.x (positive) and .y (negative)
    roll = msg.angular.z if msg.angular.z >= 0 else 0.0
    pitch = -msg.angular.z if msg.angular.z < 0 else 0.0
    
    # Convert roll, pitch, yaw to a quaternion (assuming yaw=0 for simplicity)
    quaternion = quaternion_from_euler(roll, pitch, 0)
    attitude_msg.orientation.x = quaternion[0]
    attitude_msg.orientation.y = quaternion[1]
    attitude_msg.orientation.z = quaternion[2]
    attitude_msg.orientation.w = 0
    
    # Publish the message to /mavros/setpoint_raw/attitude
    attitude_pub.publish(attitude_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('converter_vel_alt')
    
    # Create a publisher for /mavros/setpoint_raw/attitude
    attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    
    # Create a subscriber for /cmd_vel
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    
    # Spin to keep the script running
    rospy.spin()