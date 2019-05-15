#!/usr/bin/env python

import rospy
from mdas_dbw_msgs.msg import (SteeringCmd, BrakeCmd, ThrottleCmd)
from sensor_msgs.msg import Joy

class JoystickDemo:

    #publishers = {}

    def __init__(self):
        rospy.init_node('joystick_demo')
        rospy.sleep(0.5)
        rospy.loginfo(rospy.get_name() + " start")

        self.publishers = self.advertise_topics()
        rospy.Subscriber('joy', Joy, self.joy_callback)

        rospy.spin()


    def advertise_topics(self):
        publishers = {}

        publishers['throttle_cmd'] = rospy.Publisher('mdas_dbw/cmd/throttle', ThrottleCmd, queue_size=2)
        publishers['brake_cmd'] = rospy.Publisher('mdas_dbw/cmd/brake', BrakeCmd, queue_size=2)
        publishers['steering_cmd'] = rospy.Publisher('mdas_dbw/cmd/steering', SteeringCmd, queue_size=2)

        return publishers

    def joy_callback(self, joy):
        steer = SteeringCmd()
        brake = BrakeCmd()
        throttle = ThrottleCmd()
        
        speedCmd = int(joy.axes[1] * ThrottleCmd.THROTTLE_MAX) # Left stick vertical axes
        steerCmd = int(joy.axes[0] * SteeringCmd.ANGLE_MAX) * -1 # Left stick horizontal axes, reverse ordering

        throttle.pedal_cmd = speedCmd if (speedCmd > 0) else 0
        brake.pedal_cmd = speedCmd*-1 if (speedCmd < 0) else 0

        steer.steering_wheel_angle_cmd = steerCmd

        throttle.enable = False
        brake.enable = False
        steer.enable = False

        if joy.buttons[0]: # A Button, enable all commands
            throttle.enable = True
            brake.enable = True
            steer.enable = True
        if joy.buttons[1]: # B Button, enable brake
            brake.enable = True
        if joy.buttons[2]: # X Button, enable steering
            steer.enable = True
        if joy.buttons[3]: # Y Button, enable throttle
            throttle.enable = True
        
        
        throttle.header.stamp = rospy.Time.now()
        brake.header.stamp = rospy.Time.now()
        steer.header.stamp = rospy.Time.now()

        
        self.publishers['throttle_cmd'].publish(throttle)
        self.publishers['brake_cmd'].publish(brake)
        self.publishers['steering_cmd'].publish(steer)