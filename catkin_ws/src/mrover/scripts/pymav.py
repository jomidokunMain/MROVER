from pymavlink import mavutil

# Create the connection
# From topside computer
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue
    if msg.get_type() == 'COMMAND_LONG':
        rpm= msg.param1
        speed= msg.param2
        steer= msg.param3
        print("\n %s" % steer)
        # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
        #print("\nSystem status: %s" % msg.system_status)