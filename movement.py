import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

master.wait_heartbeat()
print(master)
# # master.reboot_autopilot()
# # master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0, 0)

# # msg = master.recv_match(type="COMMAND_ACK", blocking=True)
# # print(msg)

# # time.sleep(5)
# # master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

# # # msg = master.recv_match(type="NAV_CONTROLLER_OUTPUT", blocking=True)
# # msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True)
# # print(msg)



# def set_rc_channel_pwm(channel_id, pwm=1500):
#     """ Set RC channel pwm value
#     Args:
#         channel_id (TYPE): Channel ID
#         pwm (int, optional): Channel pwm value 1100-1900
#     """
#     if channel_id < 1 or channel_id > 18:
#         print("Channel does not exist.")
#         return

#     # Mavlink 2 supports up to 18 channels:
#     # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
#     rc_channel_values = [65535 for _ in range(18)]
#     rc_channel_values[channel_id - 1] = pwm
#     master.mav.rc_channels_override_send(
#         master.target_system,                # target_system
#         master.target_component,             # target_component
#         *rc_channel_values)                  # RC channel list, in microseconds.


# # Set some roll
# set_rc_channel_pwm(2, 1600)

# # Set some yaw
# set_rc_channel_pwm(4, 1600)

# # The camera pwm value sets the servo speed of a sweep from the current angle to
# #  the min/max camera angle. It does not set the servo position.
# # Set camera tilt to 45 (max) with full speed
# set_rc_channel_pwm(8, 1900)

# # Set channel 12 to 1500us
# # This can be used to control a device connected to a servo output by setting the
# # SERVO[N]_Function to RCIN12 (Where N is one of the PWM outputs)
# set_rc_channel_pwm(12, 1500)


#########################################################################################

# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Disarm
# master.arducopter_disarm() or:
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     0, 0, 0, 0, 0, 0, 0)

# # wait until disarming confirmed
# master.motors_disarmed_wait()
# print("Disarmed")
master.mav.manual_control_send(
    master.target_system,
    500,
    -500,
    250,
    500,
    0)

# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC
buttons = 1 + 1 << 3 + 1 << 7
master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    500, # 500 means neutral throttle
    0,
    buttons)
    
from pymavlink import mavutil

class RawServoOutput:
    """ A class for commanding a mavlink-controlled servo output port. """
    # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO
    CMD_SET = mavutil.mavlink.MAV_CMD_DO_SET_SERVO

    def __init__(self, master, instance, pwm_limits=(1100, 1500, 1900),
                 set_default=True):
        """ Initialise a RawServoOutput instance.

        'master' is a mavlink connection
        'instance' is the servo output (e.g. Pixhawk MAIN 1-8, AUX 9-16)
        'pwm_limits' is a tuple of min, default, and max pwm pulse-widths
            in microseconds (default (1100, 1500, 1900)).
        'set_default' is a boolean specifying whether to command the output
            to the specified default pulse-width (default True).

        """
        self.master     = master
        self.instance   = instance
        self.min_us, self._current, self.max_us = pwm_limits
        self._diff      = self.max_us - self.min_us

        if set_default:
            self.set_direct(self._current)

    def set_direct(self, us):
        """ Directly set the PWM pulse width.

        'us' is the PWM pulse width in microseconds.
            Must be between self.min_us and self.max_us.

        """
        assert self.min_us <= us <= self.max_us, "Invalid input value."

        # self.master.set_servo(self.instance, us) or:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            self.CMD_SET,
            0, # first transmission of this command
            self.instance, us,
            0,0,0,0,0 # unused parameters
        )
        self._current = us

    def set_ratio(self, proportion):
        """ Set the PWM pulse-width ratio (auto-scale between min and max).

        'proportion' is a 0-1 value, where 0 corresponds to self.min_us, and
            1 corresponds to self.max_us.

        """
        self.set_direct(proportion * self._diff + self.min_us)

    def inc(self, us=50):
        """ Increment the PWM pulse width by 'us' microseconds. """
        self.set_direct(max(self.max_us, self._current+us))

    def dec(self, us=50):
        """ Decrement the PWM pulse width by 'us' microseconds. """
        self.set_direct(min(self.min_us, self._current-us))

    def set_min(self):
        """ Set the PWM to self.min_us. """
        self.set_ratio(0)

    def set_max(self):
        """ Set the PWM to self.max_us. """
        self.set_ratio(1)

    def center(self):
        """ Set the PWM to half-way between self.min_us and self.max_us. """
        self.set_ratio(0.5)


class AuxServoOutput(RawServoOutput):
    """ A class representing a mavlink-controlled auxiliary servo output. """
    def __init__(self, master, servo_n, main_outputs=8, **kwargs):
        """ Initiliase a servo instance.

        'master' is a mavlink connection.
        'servo_n' is one of the auxiliary servo outputs, like the servo_n
            outputs with QGroundControl joystick control.
            Most likely a value between 1-3, but possibly up to 6 or 8
            depending on setup.
        'main_outputs' is the number of MAIN outputs are present on the
            autopilot device (default 8).
        **kwargs are passed to RawServoOutput.

        """
        super().__init__(master, servo_n+main_outputs, **kwargs)
        self._main_outputs = main_outputs

    @property
    def servo_n(self):
        """ Return the AUX servo number of this output. """
        return self.instance - self._main_outputs


class Gripper(AuxServoOutput):
    """ A class representing a Blue Robotics' gripper. """
    def __init__(self, master, servo_n, pwm_limits=(1100, 1500, 1900),
                 **kwargs):
        """ Initialise a Gripper instance.

        'master' is a mavlink connection.
        'servo_n' is one of the auxiliary servo outputs, like the servo_n
            outputs with QGroundControl joystick control.
            Most likely a value between 1-3, but possibly up to 6 or 8
            depending on setup.
        'pwm_limits' is a tuple of min, default, and max pwm pulse-widths
            in microseconds (default (1100, 1100, 1900)).
        **kwargs are passed to AuxServoOutput.

        """
        super().__init__(master, servo_n, pwm_limits=pwm_limits, **kwargs)

    def open(self):
        """ Command the gripper to open. """
        self.set_max()

    def close(self):
        """ Command the gripper to close. """
        self.set_min()


# if running this file as a script:
if __name__ == '__main__':
    from time import sleep

    # Connect to the autopilot (pixhawk) from the surface computer,
    #  via the companion.
    autopilot = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # Wait for a heartbeat from the autopilot before sending commands
    autopilot.wait_heartbeat()

    # create a gripper instance on servo_1 (AUX output 1)
    gripper = Gripper(autopilot, 1)

    # open and close the gripper a few times
    for _ in range(3):
        gripper.open()
        sleep(2)
        gripper.close()
        sleep(1)