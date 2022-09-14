import time
from pymavlink import mavutil

# master = mavutil.mavlink_connection('/dev/ttyACM0')
master = mavutil.mavlink_connection('/dev/ttyACM0', source_system=1)
master.wait_heartbeat()
print(master)

# master.reboot_autopilot()
master.wait_heartbeat()
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0, 0)


msg = master.recv_match(type="COMMAND_ACK")
print(msg)

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

try1 = RawServoOutput(master, 3)
try1.set_direct(1900)
time.sleep(2)
try1.set_direct(1500)
