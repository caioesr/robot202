#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import time

class claw:
    def __init__(self):
        self.arm_publisher = rospy.Publisher(
            '/joint1_position_controller/command',
            Float64,
            queue_size=1)
        self.claw_publisher = rospy.Publisher(
            '/joint2_position_controller/command',
            Float64,
            queue_size=1)
        self.arm_state = -1.5
        self.claw_state = 0
        self.time = 1

    def up_arm(self):
        arm_state = Float64()
        if(self.arm_state <= 0):
            arm_state.data = 0
        elif(self.arm_state <= 1):
            arm_state.data = 1
        elif(self.arm_state <= 1.5):
            arm_state.data = 1.5
        
        self.arm_publisher.publish(arm_state)
        time.sleep(self.time)

    def down_arm(self):
        arm_state = Float64()
        if(self.arm_state >= 1):
            arm_state.data = 1
        elif(self.arm_state >= 0):
            arm_state.data = 0
        elif(self.arm_state >= -1.5):
            arm_state.data = -1.5
        
        self.arm_publisher.publish(arm_state)
        time.sleep(self.time)

    def switch_claw_state(self):
        claw_state = Float64()
        claw_state.data = abs(1 - self.claw_state)

        self.claw_publisher.publish(claw_state)
        time.sleep(self.time)
