#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mavros_msgs.msg import RCOut
from mrover.msg import Control  #  message for the control

class Translator:
    def __init__(self):

        self.steering_sub = rospy.Subscriber('steering_angle', Float32, self.steering_callback)
        self.rc_out_sub = rospy.Subscriber('/mavros/rc/out', RCOut, self.rc_out_callback)
        # self.throttle_rpm_sub = rospy.Subscriber('throttle_rpm', Float32, self.throttle_rpm_callback)
        # self.throttle_speed_sub = rospy.Subscriber('throttle_speed', Float32, self.throttle_speed_callback)

        self.controls_pub = rospy.Publisher('rover', Control, queue_size=20)

        self.controls_msg = Control()

    def steering_callback(self, data):
        neg_range_min = 261
        state_angle = 360 - neg_range_min
        steering_mod = round(((state_angle + data.data) % 360),3)
        self.controls_msg.steer =steering_mod
        min_val = 0
        max_val = 195
        normalized_min = -1
        normalized_max = 1
        if 0 < steering_mod <= 200:
            steering_mod = steering_mod 
        else:
            steering_mod = 0.000
        self.controls_msg.steer  = ((steering_mod - min_val) / (max_val - min_val)) * (normalized_max - normalized_min) + normalized_min
        self.publish_controls()

    def rc_out_callback(self, data):
        # Assuming channel 7 is used for throttle control
        pwm_value = data.channels[6]

        # Map PWM to the desired range of -1 to 1
        normalized_throttle = 2*(pwm_value - 1000) / 600.0  # Map PWM to the range -1 to 1
        normalized_throttle = max(-1, min(1, normalized_throttle))  # Ensure the value is within [0, 1]

        self.controls_msg.throttle = normalized_throttle
        self.publish_controls()

    # def throttle_rpm_callback(self, data):
    #     # Adjust the threshold based on your application requirements
    #     if data.data > 1000:
    #         self.controls_msg.throttle = 1.0
    #     else:
    #         self.controls_msg.throttle = 0.0

    #     self.publish_controls()

    def throttle_speed_callback(self, data):
        # Add your logic to determine brake based on speed if needed
        self.controls_msg.brake = 0.0  # Placeholder, adjust as needed
        self.publish_controls()

    def publish_controls(self):
        self.controls_msg.header.stamp = rospy.Time.now()
        self.controls_msg.shift_gears = 1  # Placeholder, adjust as needed
        self.controls_msg.header.seq += 1
        self.controls_pub.publish(self.controls_msg)


if __name__ == '__main__':
    # # try:
    rospy.init_node('joystick_translatior_mrover', anonymous=True)
    t = Translator()
    rospy.spin()
    # try:
    #     translator_node = Translator()
    #     translator_node.run()
    # except rospy.ROSInterruptException:
    #     pass
