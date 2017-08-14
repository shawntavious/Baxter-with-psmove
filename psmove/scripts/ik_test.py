#! /usr/bin/python

# WORKS!!! Just for orientation
# to get correct orientation for the psmove stand the psmove upright with
# the ball facing up. Turn the wand so that the select button is facing the
# robot and the start button is facing you then press the move button
# big one on the front.

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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

# Path to import psmoveapi
sys.path.insert(0, '/home/nano/src/psmoveapi/build')

import psmove

move = psmove.PSMove()
move.enable_orientation(True)
move.reset_orientation()

class Follow(object):
	def __init__(self, limb):
		self.br = tf.TransformBroadcaster()
		self.control_limb = limb
		self.control_arm = baxter_interface.limb.Limb(limb)
		self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		self.hdr = Header(stamp = rospy.Time.now(), frame_id = '/base')
		self.pub = rospy.Publisher('psmove', PoseStamped, queue_size=10)
		print 'Getting robot state...'
		self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self.init_state = self.rs.state().enabled
		print 'Enabling robot...'
		self.rs.enable()
		self.right_gripper = baxter_interface.Gripper(limb, CHECK_VERSION)

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
		self.set_neutral()
		if not self.init_state:
			print("Disabling robot...")
			self.rs.disable()
		return True

	def get_pose(self):
		ori = move.get_orientation()
		current_pos = self.control_arm.endpoint_pose()
		# print current_pos
		# tuck position
		# {'position': Point(x=0.5781416577116227, y=-0.18403207004889116, z=0.11208451866240399), 'orientation': Quaternion(x=-0.14043347107340182, y=0.9897450915352161, z=-0.011366250506096817, w=0.02353512977774922)}
		x=0.5781416577116227
		y=-0.18403207004889116
		z=0.11208451866240399
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
						x=ori[2],
						y=ori[1],
						z=ori[3],
						w=ori[0],
					),
				),
			)
		self.pub.publish(poses)
		# show psmove orientation in rviz
		self.br.sendTransform((1.0,0.0,0.0),(qx,qy,qz,qw),rospy.Time.now(), 'psmove', '/base')
		# rviz axes
		# red = x green = y blue = z 
		return poses

	def move_arm(self, poses):
		iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(poses)
		try:
			# rospy.wait_for_service(self.ns, 5.0)
			resp = iksvc(ikreq)
			if resp.isValid[0]:
				print 'Joint solution found'
				limb_joints = dict(zip(resp.joints[0].name,resp.joints[0].position))
				self.control_arm.move_to_joint_positions(limb_joints, 0.1)
				print limb_joints
			else:
				print 'No solution'
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))

	def grab(self):
		if move.get_trigger():
			button_val = 100.0 - ((20.0/51.0) * move.get_trigger())
			self.right_gripper.set_velocity(25.0)
			self.right_gripper.command_position(button_val)


	def puppet(self):
		self.set_neutral()
		print ("Puppeting:\n"
			"  Grab %s cuff and move arm.\n"
			"  Press Ctrl-C to stop...") % (self.control_limb,)
		while not rospy.is_shutdown():
			self.move_arm(self.get_pose())
			while move.poll():
				pressed, released = move.get_button_events()
				if pressed & psmove.Btn_MOVE:
					move.reset_orientation()
				self.grab()


rospy.init_node('psmove_broadcaster', anonymous=True)
follow = Follow('right')
rospy.on_shutdown(follow.clean_shutdown)
follow.puppet()