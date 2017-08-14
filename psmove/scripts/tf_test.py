#! /usr/bin/python

# just for position

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
tracker.exposure = psmove.Exposure_LOW

while tracker.enable(move) != psmove.Tracker_CALIBRATED:
    pass

class Follow(object):
	def __init__(self, limb, offset):
		self.br = tf.TransformBroadcaster()
		self.offset = offset
		# Incase we want to put in arguments
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
		#return to normal
		self._reset_control_modes()
		self.set_neutral()
		if not self.init_state:
			print("Disabling robot...")
			self.rs.disable()
		return True

	def get_pose(self):
		pos = fusion.get_position(move)
		# qx=-0.14043347107340182 
		# qy=0.9897450915352161
		# qz=-0.011366250506096817
		# qw=0.02353512977774922
		true_x, true_y, true_z = self.offset
		# x=pos[2]/comp_x
		# y=pos[0]/comp_y
		# z=-pos[1]/comp_z
		x=(pos[2]-true_x)/comp_x
		y=(-pos[0]-true_y)/comp_y
		z=(-pos[1]-true_z)/comp_z
		# x = z
		# y = x
		# z = y
		print('Pos x:%f y:%f z:%f' %(x,y,z))
		qx=0.0
		qy=0.0
		qz=0.0
		qw=1.0
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
		# self.br.sendTransform((3,0,0),(0.0,0.0,0.0,1.0), rospy.Time.now(), '/person', '/base')
		self.br.sendTransform((x,y,z),(qx,qy,qz,qw),rospy.Time.now(), 'psmove', '/base') 
		return poses

	def grab(self):
		button_val = 100.0 - ((20.0/51.0) * move.get_trigger())
		self.right_gripper.set_velocity(25.0)
		if move.get_trigger():
			self.right_gripper.command_position(button_val)

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
			# else:
			# 	print 'No solution'
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			

	def puppet(self):
		self.set_neutral()
		while not rospy.is_shutdown():
			tracker.update_image()
			tracker.update()
			status = tracker.get_status(move)
			while move.poll(): pass
			if status == psmove.Tracker_TRACKING:
				move.set_rumble(120)
				self.move_arm(self.get_pose())
				while move.poll():
					self.grab()
			else:
				move.set_rumble(0)
			# elif status == psmove.Tracker_CALIBRATED:
			# 	print 'Not currently tracking.'
			# 	move.set_rumble(120)
			# elif status == psmove.Tracker_CALIBRATION_ERROR:
			# 	print 'Calibration error.'
			# 	move.set_rumble(120)
			# elif status == psmove.Tracker_NOT_CALIBRATED:
			# 	print 'Controller not calibrated.'
			# 	move.set_rumble(120)

def calibrate():
	print('Stand in front of the robot.')
	print('Stand the move on a flat level surface with the Select button facing the robot and the Start button facing you')
	print('While the move is correctly oriented press the large center button.')
	print('This will set the orientation.')
	keys = ['center','forward', 'up', 'right']
	circle = {}.fromkeys(keys)
	instructions = ['Hold the move vertically at your right shoulder and press the green triangle button',
					'Hold the move horizontally directly in front of you and press the green triangle']
	print 'Getting the users range of movement'
	for idx, key in enumerate(keys):
		# print instructions[idx]
		while circle[key] is None:
			tracker.update_image()
			tracker.update()
			while move.poll():
				pressed, released = move.get_button_events()
				if pressed & psmove.Btn_TRIANGLE:
					pos = fusion.get_position(move)
					x=pos[2]
					y=pos[0]
					z=-pos[1]
					circle[key] = x,y,z
					print circle[key]

	# print circle
	true_x,true_y,true_z = circle['center']

	offset = [true_x,true_y,true_z]
	true_circle = {}
	for key in circle:
		true_circle[key] = [x - y for x, y in zip(circle.get(key), offset)]
	print true_circle
	global comp_x
	radius = true_circle['forward'][0]
	comp_x = radius/0.745
	global comp_y
	radius = true_circle['right'][1]
	comp_y = radius/0.745
	global comp_z
	radius = true_circle['up'][2]
	comp_z = radius/0.745
	print('x: %f y: %f z: %f' %(comp_x, comp_y, comp_z))
	return offset

# sholder
# (-45.0065803527832, -2.2704195976257324, 6.092292785644531)
# Hold the move horizontally directly in front of you and press the green triangle
# (-30.804655075073242, -1.5124976634979248, 6.007750511169434)
# {'forward': [14.201925277709961, 0.7579219341278076, -0.08454227447509766], 'center': [0.0, 0.0, 0.0]}
# 144.353443508
# 14.2019252777


# Getting the users range of movement
# (-42.225528717041016, 0.7455440163612366, 5.89334774017334)
# (-37.58109664916992, 0.7267348766326904, 5.624297618865967)
# (-61.104270935058594, 0.7473185062408447, 12.221382141113281)
# (-40.55489730834961, 3.9894039630889893, 5.591984748840332)
# {'forward': [4.644432067871094, -0.018809139728546143, -0.26905012130737305], 'right': [1.6706314086914062, 3.2438599467277527, -0.3013629913330078], 'center': [0.0, 0.0, 0.0], 'up': [-18.878742218017578, 0.0017744898796081543, 6.328034400939941]}
# 57.9767820984
# 4.64443206787


# Getting the users range of movement
# (-32.25885009765625, -0.33146652579307556, 2.8758609294891357)
# (-13.438544273376465, 0.03389621526002884, 2.135467052459717)
# (-38.28290557861328, -0.9510905742645264, 7.527212142944336)
# (-31.274578094482422, 8.357751846313477, 3.5443503856658936)
# {'forward': [18.820305824279785, 0.3653627410531044, -0.740393877029419], 'right': [0.9842720031738281, 8.689218372106552, 0.6684894561767578], 'center': [0.0, 0.0, 0.0], 'up': [-6.024055480957031, -0.6196240484714508, 4.6513512134552]}


rospy.init_node('psmove_broadcaster', anonymous=True)
follow = Follow('right', calibrate())
rospy.on_shutdown(follow.clean_shutdown)
follow.puppet()