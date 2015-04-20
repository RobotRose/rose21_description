#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import random


joints = [
          "BR_caster"
          ,"BL_caster"
          ,"FR_caster"
          ,"FL_caster"
          ,"lift_mid_bottom_joint"
          ,"lift_mid_box_joint"
          ,"neck_fixed_joint"
          ,"neck_pan_joint"
          ,"neck_tilt_joint"
          ,"hokuyo_joint"
          ]

def r():
    return random.random()

def create():
    js = JointState()
    js.name = joints
    js.header.stamp = rospy.Time.now()

    js.position = [r() for j in joints]
    js.velocity = [0 for j in joints]
    js.effort = [0 for j in joints]
    return js

rospy.init_node("dummy_joint_publisher")

pub = rospy.Publisher("/joint_states", JointState)

while not rospy.is_shutdown():
    pub.publish(create())
    rospy.sleep(0.1)
