#! /usr/bin/python

# baxter will mirror your moves

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

tracker = psmove.PSMoveTracker()
tracker.set_mirror(True)

fusion = psmove.PSMoveFusion(tracker, 1, 1000)

move = psmove.PSMove()
move.enable_orientation(True)
move.reset_orientation()
tracker.exposure = psmove.Exposure_LOW

last_x=0.0
last_y=0.0
last_z=0.0
last_qx=0.0
last_qy=0.0
last_qz=0.0
last_qw=0.0

while tracker.enable(move) != psmove.Tracker_CALIBRATED:
    pass

class Follow(object):
	def __init__(self, matrix):
		self.br = tf.TransformBroadcaster()
		self.matrix = matrix
		self.right_arm = baxter_interface.Limb('right')
		self.left_arm = baxter_interface.Limb('left')
		self.right_ns = "ExternalTools/right/PositionKinematicsNode/IKService"
		self.left_ns = "ExternalTools/left/PositionKinematicsNode/IKService"
		self.hdr = Header(stamp = rospy.Time.now(), frame_id = '/base')
		self.pub = rospy.Publisher('psmove', PoseStamped, queue_size=10)

		print 'Getting robot state...'
		self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self.init_state = self.rs.state().enabled
		print 'Enabling robot...'
		self.rs.enable()
		self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
		self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)

	def _reset_control_modes(self):
		for _ in xrange(100):
			if rospy.is_shutdown():
				return False
			self.right_arm.exit_control_mode()
			self.left_arm.exit_control_mode()
		return True

	def set_neutral(self):
		print("Moving to neutral pose...")
		self.right_arm.move_to_neutral()
		self.left_arm.move_to_neutral()
		print 'Done'

	def clean_shutdown(self):
		print("\nExiting example...")
		#return to normal
		self._reset_control_modes()
		self.set_neutral()
		if not self.init_state:
			print("Disabling robot...")
			self.rs.disable()
			print 'Done'
		return True

	def get_pose(self, pos_pause, ori_pause):
		ori = move.get_orientation()
		pos = fusion.get_position(move)
		true_x, true_y, true_z = self.matrix[0]
		comp_x, comp_y, comp_z = self.matrix[1]
		x=(pos[2]-true_x)/comp_x
		y=(pos[0]-true_y)/comp_y
		z=((-pos[1]-true_z)/comp_z)
		qx=ori[1]
		qy=ori[2]
		qz=ori[3]
		qw=ori[0]
		
		global last_x
		global last_y
		global last_z
		global last_qx
		global last_qy
		global last_qz
		global last_qw
		if pos_pause: x,y,z = last_x, last_y, last_z
		else: last_x, last_y, last_z = x,y,z
		if ori_pause: qx,qy,qz,qw = last_qx, last_qy, last_qz, last_qw
		else: last_qx, last_qy, last_qz, last_qw = qx,qy,qz,qw

		poses = PoseStamped(
			header=self.hdr,
				pose=Pose(
					position=Point(
						x,y,z,
					),
						orientation=Quaternion(
						qx,qy,qz,qw
					),
				),
			)
		self.pub.publish(poses)
		self.br.sendTransform((x,y,z),(qx,qy,qz,qw),rospy.Time.now(), '/psmove', '/base') 
		return poses

	def grab(self, arm):
		button_val = 100.0 - ((20.0/51.0) * move.get_trigger())
		if arm == self.right_arm:
			self.right_gripper.set_velocity(25.0)
			if move.get_trigger():
				self.right_gripper.command_position(button_val)
		elif arm == self.left_arm:
			self.left_gripper.set_velocity(25.0)
			if move.get_trigger():
				self.left_gripper.command_position(button_val)

	def move_arm(self, poses, arm):
		if arm == self.right_arm:
			iksvc = rospy.ServiceProxy(self.right_ns, SolvePositionIK)
			ikreq = SolvePositionIKRequest()
			ikreq.pose_stamp.append(poses)
			try:
				resp = iksvc(ikreq)
				if resp.isValid[0]:
					# print 'Joint solution found'
					limb_joints = dict(zip(resp.joints[0].name,resp.joints[0].position))
					self.right_arm.move_to_joint_positions(limb_joints, 0.1)
					# print limb_joints
			except (rospy.ServiceException, rospy.ROSException), e:
				rospy.logerr("Service call failed: %s" % (e,))
		elif arm == self.left_arm:
			iksvc = rospy.ServiceProxy(self.left_ns, SolvePositionIK)
			ikreq = SolvePositionIKRequest()
			ikreq.pose_stamp.append(poses)
			try:
				resp = iksvc(ikreq)
				if resp.isValid[0]:
					# print 'Joint solution found'
					limb_joints = dict(zip(resp.joints[0].name,resp.joints[0].position))
					self.arm.move_to_joint_positions(limb_joints, 0.1)
					# print limb_joints
			except (rospy.ServiceException, rospy.ROSException), e:
				rospy.logerr("Service call failed: %s" % (e,))
				

	def puppet(self):
		arm = self.left_arm
		self.set_neutral()
		pos_pause = False
		ori_pause = False
		while not rospy.is_shutdown():
			tracker.update_image()
			tracker.update()
			status = tracker.get_status(move)
			while move.poll():
				pressed, released = move.get_button_events()
				if pressed & psmove.Btn_MOVE:
					move.reset_orientation()
				elif pressed & psmove.Btn_SQUARE:
					pos_pause = not pos_pause
				elif pressed & psmove.Btn_CROSS:
					ori_pause = not ori_pause
				elif pressed & psmove.Btn_CIRCLE:
					self.set_neutral()
				elif pressed & psmove.Btn_SELECT:
					if arm == self.left_arm:
						arm = self.right_arm
					elif arm == self.right_arm:
						arm = self.left_arm
				elif pressed & psmove.Btn_PS:
					self.clean_shutdown()
					sys.exit(0)
			if status == psmove.Tracker_TRACKING:
				move.set_rumble(0)
				self.move_arm(self.get_pose(pos_pause, ori_pause), arm)
				while move.poll():
					self.grab(arm)
			elif status == psmove.Tracker_CALIBRATED:
				print 'Not currently tracking.'
				move.set_rumble(120)
			elif status == psmove.Tracker_CALIBRATION_ERROR:
				print 'Calibration error.'
				move.set_rumble(120)
			elif status == psmove.Tracker_NOT_CALIBRATED:
				print 'Controller not calibrated.'
				move.set_rumble(120)

def orient():
	print('Stand in front of the robot.')
	print('Stand the move on a flat level surface with the Select button facing the robot and the Start button facing you')
	print('While the move is correctly oriented press the large center button.')
	print('This will set the orientation.')
	flag = False
	while not flag:
		while move.poll():
			pressed, released = move.get_button_events()
			if pressed & psmove.Btn_MOVE:
				move.reset_orientation()
				print 'Orientation set.'
				flag = True

def calibrate():
	# The farthest the arm can reach in meters
	max_reach = 0.9
	orient()
	keys = ['center','forward', 'up', 'right']
	circle = {}.fromkeys(keys)
	instructions = ['Hold the move vertically at the center of your chest and press the green triangle button',
					'Hold the move horizontally directly in front of you and press the green triangle',
					'Hold the move vertically above your head and press the green triangle',
					'Hold the move horizontally directly to your right and press the green triangle']
	print 'Getting the users range of movement'
	for idx, key in enumerate(keys):
		print instructions[idx]
		while circle[key] is None:
			tracker.update_image()
			tracker.update()
			status = tracker.get_status(move)
			if status == psmove.Tracker_TRACKING:
				move.set_rumble(0)
				while move.poll():
					pressed, released = move.get_button_events()
					if pressed & psmove.Btn_TRIANGLE:
						pos = fusion.get_position(move)
						x=pos[2]
						y=pos[0]
						z=-pos[1]
						circle[key] = x,y,z
						print circle[key]
			else:
				move.set_rumble(120)

	# print circle
	true_x,true_y,true_z = circle['center']
	comps = []
	offset = [true_x,true_y,true_z]
	true_circle = {}
	for key in circle:
		true_circle[key] = [x - y for x, y in zip(circle.get(key), offset)]
	print true_circle
	radius = true_circle['forward'][0]
	comps.append(radius/max_reach)
	print comps
	radius = true_circle['right'][1]
	comps.append(radius/max_reach)
	radius = true_circle['up'][2]
	comps.append(radius/max_reach)
	matrix = [offset,comps]
	return matrix

rospy.init_node('psmove_broadcaster', anonymous=True)
follow = Follow(calibrate())
rospy.on_shutdown(follow.clean_shutdown)
follow.puppet()