#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

class ImuToBaseLink:
    def __init__(self):
        rospy.init_node('imu_to_base_link', anonymous=True)
        
        self.imu_data = None
        self.br = tf.TransformBroadcaster()
        
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.rate = rospy.Rate(50)  # Adjust the rate as needed

    def imu_callback(self, data):
        self.imu_data = data

    def broadcast_transform(self):
        if self.imu_data is not None:
            orientation_q = self.imu_data.orientation
            
            # Quaternion (x, y, z, w)
            quat = (
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            )
            
            # Broadcast transform (no translation, only rotation from IMU data)
            self.br.sendTransform(
                (0, 0, 0),  # Translation
                quat,       # Rotation (quaternion)
                rospy.Time.now(),
                'base_link',
                'odom'  # Assuming 'odom' is the parent frame
            )

    def run(self):
        while not rospy.is_shutdown():
            self.broadcast_transform()
            self.rate.sleep()

if __name__ == '__main__':
    imu_to_base_link = ImuToBaseLink()
    imu_to_base_link.run()
