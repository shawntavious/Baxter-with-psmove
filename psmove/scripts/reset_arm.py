#! /usr/bin/python

# Script that allows you to easily manipulate the frames and 
# to check the psmove transform and pose on a static robot. 
# Basiclly the transform and pose moves in rviz the arms don't.

import roslib
import rospy
import tf
import os
import signal
import numpy as np
import sys
from ctypes import *
import baxter_interface
from baxter_interface import CHECK_VERSION

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

sys.path.insert(0, '/home/nano/src/psmoveapi/build')

import psmove

move = psmove.PSMove()
move.enable_orientation(True)
move.reset_orientation()

class Follow(object):
	def __init__(self):
		# Change frame here
		self.frame = '/base'
		self.br = tf.TransformBroadcaster()
		self.control_limb = 'right'
		self.control_arm = baxter_interface.Limb(self.control_limb)
		self.hdr = Header(stamp = rospy.Time.now(), frame_id = self.frame)
		self.pub = rospy.Publisher('psmove', PoseStamped, queue_size=10)
		print 'Getting robot state...'
		self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self.init_state = self.rs.state().enabled
		print 'Enabling robot...'
		self.rs.enable()

	def _reset_control_modes(self):
		for _ in xrange(100):
			if rospy.is_shutdown():
				return False
			self.control_arm.exit_control_mode()
		return True

	def set_neutral(self):
		print("Moving to neutral pose...")
		self.control_arm.move_to_neutral()

	def clean_shutdown(self):
		print("\nExiting example...")
		self._reset_control_modes()
		if not self.init_state:
			print("Disabling robot...")
			self.rs.disable()
		return True

	def get_pose(self):
		ori = move.get_orientation()
		x=0.0
		y=0.0
		z=0.0
		qx=ori[1]
		qy=ori[2]
		qz=ori[3]
		qw=ori[0]
		poses = PoseStamped(
			header=self.hdr,
				pose=Pose(
					position=Point(
						x,
						y,
						z,
						),
						orientation=Quaternion(
						x=ori[1],
						y=ori[2],
						z=ori[3],
						w=ori[0],
					),
				),
			)
		self.pub.publish(poses)
		self.br.sendTransform((x,y,z),(qx,qy,qz,qw),rospy.Time.now(), 'psmove1', self.frame) 
		return poses

	def puppet(self):
		self.set_neutral()
		while not rospy.is_shutdown():
			self.get_pose()
			while move.poll():
				pressed, released = move.get_button_events()
				if pressed & psmove.Btn_MOVE:
					move.reset_orientation()


rospy.init_node('psmove_broadcaster', anonymous=True)
follow = Follow()
rospy.on_shutdown(follow.clean_shutdown)
follow.puppet()