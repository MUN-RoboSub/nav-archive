import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

master.wait_heartbeat()

master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0, 0)


msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

# master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0, 0)
# while True:
#     try:
#         print(master.recv_match().to_dict())
#     except:
#         pass
#     time.sleep(0.1)