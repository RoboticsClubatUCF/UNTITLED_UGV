#!/usr/bin/env python

""" Allows us to send commands directly to the motor controllers"""

# ros imports
import rospy

# python imports
import serial
import math
from typing import Tuple

# message imports
import std_msgs.msg as std
import geometry_msgs.msg as geom
import ugv_msg.msg as ugv

AUTO_SWITCH = False
METERS_PER_REV = WHEEL_RADIUS * math.pi * 2
REVS_PER_METER = 1 / METERS_PER_REV


def rc_callback(message, args):
    # Auto-nav mode
    if message.switch_d == "True":
        args[2] = True
        return

    left_rpm =int((message.right_x  - 1500)*(0.4))
    right_rpm =int((message.left_x  - 1500)*(0.4))

    string = "!M " + str(right_rpm) + " " + str(left_rpm) + "\r"
    print(string)
    encoded = string.encode('utf-8')
    args[0].write(encoded)
    args[1].write(encoded)
    args[2] = False
    

def cmd_vel_cb(cmd_vel, args):

    # Auto-nav mode is off
    if args[2] == False:
        return
    
    wheel_base = 0.67
    left_velocity =  cmd_vel.linear.x - 0.5*cmd_vel.angular.z*wheel_base;
    right_velocity = cmd_vel.linear.x + 0.5*cmd_vel.angular.z*wheel_base;


    # convert m/s to RPM
    left_rpm = left_velocity * REVS_PER_METER * 60
    right_rpm = right_velocity * REVS_PER_METER * 60


    # Serial Write
    string = "!M " + str(right_rpm) + " " + str(left_rpm) + "\r"
    encoded = string.encode('utf-8')
    args[0].write(encoded)
    args[1].write(encoded)





def main():     

    rospy.init_node('motor_controller_bridge', anonymous=True, log_level=rospy.DEBUG)
    _port = '/dev/ttyACM0'
    ser1 = serial.Serial(
        port=_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    rospy.logdebug("Serial Connection established on {}".format(_port))

    _port = '/dev/ttyACM1'
    ser2 = serial.Serial(
        port=_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    rospy.logdebug("Serial Connection established on {}".format(_port))

    # Subscribers
    rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=rc_callback,callback_args=(ser1,ser2,AUTO_SWITCH))
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', geom.Twist, callback=cmd_vel_cb, callback_args=(ser1,ser2, AUTO_SWITCH))
    
    
    while not rospy.is_shutdown():
        x = 1

if __name__=='__main__':
    main()