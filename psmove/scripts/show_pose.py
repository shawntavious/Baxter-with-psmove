#! /usr/bin/python

# prints current pose of an arm
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

rospy.init_node('show_pose', anonymous=True)
limb = baxter_interface.Limb('left')
pose = limb.endpoint_pose()
print pose