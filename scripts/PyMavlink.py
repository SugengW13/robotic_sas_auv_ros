import sys
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

class PyMavlink():
    def __init__(self, master):
        self.master = master
        self.bootTime = time.time()
        self.rcValue = [1500] * 8

    def armDisarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('Armed!')

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        self.master.motors_disarmed_wait()
    
    def arm(self):
        self.master.arducopter_arm()
    
    def disarm(self):
        self.master.arducopter_disarm()

    def setMode(self, mode):
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)


        modeId = self.master.mode_mapping()[mode]

        self.master.set_mode(modeId)
    
    def setRcValue(self, channel, pwm):
        self.rcValue[channel - 1] = pwm

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *self.rcValue
        )

    def setDepth(self, depth):
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.bootTime)),
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=(
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ),
            lat_int=0, lon_int=0, alt=depth,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        )
    
    def setHeading(self, degree, loop):
        for _ in range(0, loop, 1):
            self.master.mav.set_attitude_target_send(
                int(1e3 * (time.time() - self.bootTime)),
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
                QuaternionBase([math.radians(angle) for angle in (0, 0, degree)]),
                0, 0, 0, 0
            )