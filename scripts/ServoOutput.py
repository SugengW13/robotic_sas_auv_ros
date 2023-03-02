from pymavlink import mavutil

class RawServoOutput:
    CMD_SET = mavutil.mavlink.MAV_CMD_DO_SET_SERVO

    def __init__(self, master, instance, pwm_limits=(1100, 1500, 1900), set_default=True):
        self.master     = master
        self.instance   = instance
        self.min_us, self._current, self.max_us = pwm_limits
        self._diff      = self.max_us - self.min_us

        if set_default:
            self.set_direct(self._current)

    def set_direct(self, us):
        assert self.min_us <= us <= self.max_us, "Invalid input value."

        self.master.set_servo(self.instance, us)
        # self.master.mav.command_long_send(
        #     self.master.target_system, self.master.target_component,
        #     self.CMD_SET,
        #     0, # first transmission of this command
        #     self.instance, us,
        #     0,0,0,0,0 # unused parameters
        # )
        self._current = us

    def set_ratio(self, proportion):
        self.set_direct(proportion * self._diff + self.min_us)

    def inc(self, us=50):
        self.set_direct(max(self.max_us, self._current+us))

    def dec(self, us=50):
        self.set_direct(min(self.min_us, self._current-us))

    def set_min(self):
        self.set_ratio(0)

    def set_max(self):
        self.set_ratio(1)

    def center(self):
        self.set_ratio(0.5)



class AuxServoOutput(RawServoOutput):
    def __init__(self, master, servo_n, main_outputs=8, **kwargs):
        super().__init__(master, servo_n+main_outputs, **kwargs)

        self._main_outputs = main_outputs

class Gripper(AuxServoOutput):
    def __init__(self, master, servo_n, pwm_limits=(1100, 1100, 1500), **kwargs):
        super().__init__(master, servo_n, pwm_limits=pwm_limits, **kwargs)

    def open(self):
        self.set_max()
    
    def close(self):
        self.set_min()