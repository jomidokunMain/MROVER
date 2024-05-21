#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import Mavlink
from mrover.msg import CustomCommand
from struct import unpack
from mrover.msg import Encoding

class Listener:
    def __init__(self):
        self.mavlink_sub =rospy.Subscriber('/mavlink/from', Mavlink, self.mavlink_callback)
        self.custom_command_publisher = rospy.Publisher('/encode_val', CustomCommand, queue_size=10)


    def mavlink_callback(self, data):
        # Check if the received message is a COMMAND_LONG message
        if data.msgid == 76:  # The msgid for COMMAND_LONG in MAVLink
            # Parse the data and extract relevant information
            payload_bytes = bytearray()
            for val in data.payload64:
                payload_bytes.extend(val.to_bytes(8, byteorder='little'))
        
            # Unpack the fixed-length portion of the payload
            fixed_length = 5  # target_system, target_component, command, confirmation, param_count
            fixed_payload = unpack('<BBBBL', payload_bytes[:fixed_length])
            
            # Extract relevant information from the unpacked fixed-length payload
            target_system, target_component, command, confirmation, param_count = fixed_payload
            
            # Determine the total number of parameters
            param_length = len(payload_bytes) - fixed_length
            num_params = param_length // 4  # Each parameter is a float (4 bytes)
            
            # Unpack the variable-length portion of the payload (parameters)
            param_format = '<' + 'f' * num_params
            params = unpack(param_format, payload_bytes[fixed_length:])
            
            # Create a CustomCommand message
            command_msg = CustomCommand()
            command_msg.target_system = target_system
            command_msg.target_component = target_component
            command_msg.command = command
            command_msg.param1 = params[0] if num_params >= 1 else 0.0
            command_msg.param2 = params[1] if num_params >= 2 else 0.0
            command_msg.param3 = params[2] if num_params >= 3 else 0.0
            command_msg.param4 = params[3] if num_params >= 4 else 0.0
            command_msg.param5 = params[4] if num_params >= 5 else 0.0
            command_msg.param6 = params[5] if num_params >= 6 else 0.0
            command_msg.param7 = params[6] if num_params >= 7 else 0.0


            # command_msg = Encoding()
            # command_msg.command = data.payload64[0]  # Command ID
            # command_msg.target_system = data.payload64[1]  # Target System
            # command_msg.target_component = data.payload64[2]  # Target Component

            # command_msg.ENthrottle = payload[6]  # Parameter 1
            # command_msg.ENsteer = 0 # Parameter 2
            # command_msg.param3 = data.payload64[5]  # Parameter 3
            # command_msg.param4 = data.payload64[6]  # Parameter 4
            # command_msg.param5 = data.payload64[7]  # Parameter 5
            # command_msg.param6 = data.payload64[8]  # Parameter 6
            # command_msg.param7 = data.payload64[9]  # Parameter 7

            # Publish your custom message
            self.custom_command_publisher.publish(command_msg)

        
        

if __name__ == '__main__':
    rospy.init_node('encoder_topic', anonymous=True)
    l =Listener()
    rospy.spin()
    