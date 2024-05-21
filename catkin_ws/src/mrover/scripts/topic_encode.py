#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from mavros_msgs.msg import WheelOdomStamped
from pymavlink import mavutil

class MavlinkToROSNode:
    def __init__(self):
        

        self.master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

        self.rpm_pub = rospy.Publisher('/mavros/wheel_odometry/rpm', WheelOdomStamped, queue_size=20)
        self.steer_pub = rospy.Publisher('/steering_angle', Float32, queue_size=20)

        self.rate = rospy.Rate(1000)  # 10 Hz

    def run(self):
        while not rospy.is_shutdown():
            msg = self.master.recv_match(blocking=True)
            if not msg:
                continue
            if msg.get_type() == 'COMMAND_LONG':
                rpm = msg.param1
                speed = msg.param2
                steer = msg.param3

                # Publish RPM
                rpm_msg = WheelOdomStamped()
                rpm_msg.header.stamp = rospy.Time.now()
                rpm_msg.data = [rpm]
                self.rpm_pub.publish(rpm_msg)

                # Publish steering angle
                self.steer_pub.publish(steer)

                # rospy.loginfo("RPM: {}, Steering Angle: {}".format(rpm, steer))

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('topic_encode', anonymous=True)
    mavlink_to_ros_node = MavlinkToROSNode()
    mavlink_to_ros_node.run()
    rospy.spin()

