#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed

def active_callback(msg):
    rospy.loginfo(f"[ACTIVE] /EXIT_PARK/active = {msg.data}")

def enable_callback(msg):
    rospy.loginfo(f"[PACMOD ENABLE] /pacmod/as_rx/enable = {msg.data}")

def gear_callback(msg):
    rospy.loginfo(f"[GEAR] ui16_cmd = {msg.ui16_cmd}, enable={msg.enable}, clear={msg.clear}, ignore={msg.ignore}")

def brake_callback(msg):
    rospy.loginfo(f"[BRAKE] f64_cmd = {msg.f64_cmd}, enable={msg.enable}, clear={msg.clear}, ignore={msg.ignore}")

def accel_callback(msg):
    rospy.loginfo(f"[ACCEL] f64_cmd = {msg.f64_cmd}, enable={msg.enable}, clear={msg.clear}, ignore={msg.ignore}")

def turn_callback(msg):
    rospy.loginfo(f"[TURN SIGNAL] ui16_cmd = {msg.ui16_cmd}, enable={msg.enable}, clear={msg.clear}, ignore={msg.ignore}")

def steer_callback(msg):
    rospy.loginfo(f"[STEER] angular_position = {msg.angular_position}")

def listener():
    rospy.init_node('pacmod_debug_listener', anonymous=True)

    rospy.Subscriber("/EXIT_PARK/active", Bool, active_callback)
    rospy.Subscriber("/pacmod/as_rx/enable", Bool, enable_callback)
    rospy.Subscriber("/pacmod/as_rx/shift_cmd", PacmodCmd, gear_callback)
    rospy.Subscriber("/pacmod/as_rx/brake_cmd", PacmodCmd, brake_callback)
    rospy.Subscriber("/pacmod/as_rx/accel_cmd", PacmodCmd, accel_callback)
    rospy.Subscriber("/pacmod/as_rx/turn_cmd", PacmodCmd, turn_callback)
    rospy.Subscriber("/pacmod/as_rx/steer_cmd", PositionWithSpeed, steer_callback)

    rospy.loginfo("Listening for PACMOD command messages...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
