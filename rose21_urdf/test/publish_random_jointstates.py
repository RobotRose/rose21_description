#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import random

joints = ["lift_front_bottom", "lift_front_top", "lift_back_bottom_left", "lift_back_bottom_right", "neck_pan_joint", "link0", "link1", "link2", "link3", "link4", "link5"]

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
	rospy.sleep(0.5)
