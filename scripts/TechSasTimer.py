import math
import time
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rc_value = [1500] * 8
current_heading = 0
ALT_HOLD_MODE = 2

def arm():
    master.arducopter_arm()

def disarm():
    master.arducopter_disarm()

def is_alt_hold(mode):
    try:
        return bool(master.wait_heartbeat().custom_mode == mode)
    except:
        return False

def set_target_depth(depth):
    master.mav.set_position_target_global_int_send(
        0,     
        0, 0,   
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
        0b0000111111111000,
        0,0, depth,
        0 , 0 , 0 , # x , y , z velocity in m/ s ( not used )
        0 , 0 , 0 , # x , y , z acceleration ( not supported yet , ignored in GCS Mavlink )
        0 , 0
    ) # yaw , yawrate ( not supported yet , ignored in GCS Mavlink )

def get_heading():
    message = master.recv_match(type='ATTITUDE', blocking=True)
    print('Get Head')
    return int(message.yaw * 180 / math.pi)

def set_target_attitude(roll, pitch, yaw, control_yaw=True):
    bitmask = (1<<6 | 1<<3)  if control_yaw else 1<<6

    master.mav.set_attitude_target_send(
        0,     
        0, 0,   
        bitmask,
        QuaternionBase([math.radians(roll), math.radians(pitch), math.radians(yaw)]), # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        0, #roll rate
        0, #pitch rate
        0, 0
    )    # yaw rate, thrust 

def set_rc_value(channel, pwm):
    if channel < 1 or channel > 18:
        print("Channel does not exist.")
        return

    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values
    )                  # RC channel list, in microseconds.

def main():
    global current_heading

    print('arming')
    # arm()

    time.sleep(1)

    while not is_alt_hold(ALT_HOLD_MODE):
        master.set_mode('ALT_HOLD')

    time.sleep(1)
    
    print('Set Depth')
    set_target_depth(-0.4)
    
    time.sleep(3)

    current_heading = get_heading()
    print('Current Heading', current_heading)

    time.sleep(3)

    print('Set Heading')
    for _ in range(3):
        set_target_attitude(0, 0, current_heading)
        time.sleep(1)

    time.sleep(3)

    # print('Forward')
    # set_rc_value(5, 1650)
    # time.sleep(5)
    # set_rc_value(5, 1500)

    # time.sleep(3)

    current_heading = get_heading()
    print('Current Heading', current_heading)

    print('Set Heading')
    for _ in range(3):
        set_target_attitude(0, 0, current_heading + 90)
        time.sleep(1)
    
    time.sleep(3)

    # print('Forward')
    # set_rc_value(5, 1650)
    # time.sleep(3)
    # set_rc_value(5, 1500)

    # time.sleep(3)

    current_heading = get_heading()
    print('Current Heading', current_heading)

    print('Set Heading')
    for _ in range(3):
        set_target_attitude(0, 0, current_heading - 90)
        time.sleep(1)

    time.sleep(3)

    print('disarming')
    # disarm()

if __name__ == '__main__':
    main()