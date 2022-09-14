import time
from pymavlink import mavutil

# Create the connection
# Need to provide the serial port and baudrate


# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# # x = mavutil.mavlink_connection("udpin:localhost:14550")

# master.wait_heartbeat()

# print(master.target_system, master.target_component)

master = mavutil.mavlink_connection('/dev/ttyACM0')

# Make sure the connection is valid
master.wait_heartbeat()

# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        print("ok")
        pass
    time.sleep(0.1)

# while True:
#     try:
#         print(master.recv_match().to_dict())
#     except:
#         pass
#     time.sleep(0.1)

# Restart the ArduSub board !
# master.reboot_autopilot()