#! /usr/bin/python

'''
Richard Sweetman 8/8/17
rsweetman@knights.ucf.edu
'''
'''
WARNING: NOT TESTED OUTSIDE OF SIMULATOR. USE CAUTION.
Baxter robot will mirror your movements. Make sure the tracker is tracking
by running ./test_tracker in the src/psmoveapi/build directory. If the box
isn't tracking the ball run it again until it works.
Then make sure you are back far enough so that the ball is never out of the
camera's view and follow the calibration instructions given in the terminal.
'''

import rospy
import tf
import os
import sys
import baxter_interface
from baxter_interface import CHECK_VERSION

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

'''Needed for baxter's IK'''
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

'''Path to import psmoveapi'''
sys.path.insert(0, '/home/nano/src/psmoveapi/build')

'''API documentation
https://thp.io/2012/thesis/thesis.pdf'''
import psmove

'''Allows for standard tracking in a 2D environment'''
tracker = psmove.PSMoveTracker()
tracker.set_mirror(True)

'''Sensor fusion module for 3D tracking'''
fusion = psmove.PSMoveFusion(tracker, 1, 1000)

move = psmove.PSMove()
move.enable_orientation(True)
move.reset_orientation()
tracker.exposure = psmove.Exposure_LOW

'''Globals needed so we can freeze position and orientation'''
last_x=0
last_y=0
last_z=0
last_qx=0
last_qy=0
last_qz=0
last_qw=0

while tracker.enable(move) != psmove.Tracker_CALIBRATED:
    pass


class Follow(object):
	'''inits broadcasters, publishers, limb, gripper, and so on'''
	def __init__(self, limb, matrix):
		self.br = tf.TransformBroadcaster()
		'''holds the various translations needed to make the arm work in rviz'''
		self.matrix = matrix
		self.control_arm = baxter_interface.limb.Limb(limb)
		self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		self.hdr = Header(stamp = rospy.Time.now(), frame_id = '/base')
		self.pub = rospy.Publisher('psmove', PoseStamped, queue_size=10)

		print 'Getting robot state...'
		self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self.init_state = self.rs.state().enabled
		print 'Enabling robot...'
		self.rs.enable()
		self.gripper = baxter_interface.Gripper(limb, CHECK_VERSION)

	def _reset_control_modes(self):
		for _ in xrange(100):
			if rospy.is_shutdown():
				return False
			self.control_arm.exit_control_mode()
		return True

	'''Resets the arm to neutral, 15 second timeout'''
	def set_neutral(self):
		print("Moving to neutral pose...")
		self.control_arm.move_to_neutral()
		print 'Done'

	def clean_shutdown(self):
		print("\nExiting example...")
		self._reset_control_modes()
		self.set_neutral()
		if not self.init_state:
			print("Disabling robot...")
			self.rs.disable()
			print 'Done'
		return True

	def get_pose(self, pos_pause, ori_pause):
		'''Ori works very accurately'''
		ori = move.get_orientation()
		'''Pos not as accurate.'''
		pos = fusion.get_position(move)
		true_x, true_y, true_z = self.matrix[0]
		comp_x, comp_y, comp_z = self.matrix[1]
		
		'''Does the math needed to compress the cordinates so they fit within
		baxters range of motion as well as smooth out the movements a little'''
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

		'''locks the coords if the user pauses the robots movement'''
		if pos_pause: x,y,z = last_x, last_y, last_z
		else: last_x, last_y, last_z = x,y,z
		if ori_pause: qx,qy,qz,qw = last_qx, last_qy, last_qz, last_qw
		else: last_qx, last_qy, last_qz, last_qw = qx,qy,qz,qw

		'''Puts the pose in a format that IK can use'''
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
		'''published only so it can be viewed on rviz'''
		self.pub.publish(poses)
		self.br.sendTransform((x,y,z),(qx,qy,qz,qw),rospy.Time.now(), '/psmove', '/base') 
		return poses

	def grab(self):
		'''Puts the button values on a 0-100 scale instead of a 0-255 scale
		so we can pass those values to baxter'''
		'''Allows user to close gripper by relation to how far they pull the trigger
		WARNING: MAY REQUIRE FORCE CALIBRATION.'''
		if move.get_trigger():
			button_val = 100.0 - ((20.0/51.0) * move.get_trigger())
			self.gripper.set_velocity(25.0)
			self.gripper.command_position(button_val)

	def move_arm(self, poses):
		'''Calls the IK services'''
		iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(poses)
		try:
			'''Does the hard work of solving the joint angles for a particular position'''
			resp = iksvc(ikreq)
			if resp.isValid[0]:
				# print 'Joint solution found'
				limb_joints = dict(zip(resp.joints[0].name,resp.joints[0].position))
				'''Function that actually moves the arm to limb_joints, waits 0.1 seconds for move to finish'''
				self.control_arm.move_to_joint_positions(limb_joints, 0.1)
				# print limb_joints
				move.set_rumble(0)
			else:
				move.set_rumble(120)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			
	def puppet(self):
		self.set_neutral()
		pos_pause = False
		ori_pause = False

		while not rospy.is_shutdown():
			'''Constantly updates the tracker'''
			tracker.update_image()
			tracker.update()
			status = tracker.get_status(move)

			if status == psmove.Tracker_TRACKING:
				'''Resets the rumble'''
				move.set_rumble(0)
				'''Gets buttons'''
				while move.poll():
					pressed, released = move.get_button_events()
					'''Square locks the position of the robot, X locks the orientation'''
					if pressed & psmove.Btn_SQUARE:
						pos_pause = not pos_pause
					elif pressed & psmove.Btn_CROSS:
						ori_pause = not ori_pause
					elif pressed & psmove.Btn_MOVE:
						move.reset_orientation()
					elif pressed & psmove.Btn_CIRCLE:
						self.set_neutral()
					elif pressed & psmove.Btn_PS:
						self.clean_shutdown()
						sys.exit(0)
				'''Moves the arm to the pose from get pose'''
				self.move_arm(self.get_pose(pos_pause, ori_pause))
				self.grab()
			elif status == psmove.Tracker_CALIBRATED:
				print 'Not currently tracking.'
				move.set_rumble(120)
			elif status == psmove.Tracker_CALIBRATION_ERROR:
				print 'Calibration error.'
				move.set_rumble(120)
			elif status == psmove.Tracker_NOT_CALIBRATED:
				print 'Controller not calibrated.'
				move.set_rumble(120)

'''Provides instructions on how to properly set the orientation'''
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

'''Calibrates the distance. This function needs improvement'''
def calibrate():
	'''The farthest the arm can reach in meters'''
	max_reach = 0.745
	orient()
	keys = ['center','forward', 'up', 'right']
	circle = {}.fromkeys(keys)
	instructions = ['Hold the move vertically at the center of your chest and press the green triangle button',
					'Hold the move horizontally directly in front of you and press the green triangle',
					'Hold the move vertically above your head and press the green triangle',
					'Hold the move horizontally directly to your right and press the green triangle']
	print 'Getting the users range of movement'
	'''Gets the users range of motion so it can be mapped to baxter'''
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
					'''Triangle when pressed gets the position from move'''
					if pressed & psmove.Btn_TRIANGLE:
						pos = fusion.get_position(move)
						'''psmove api's x,y,z positions are set up differently
						then in rviz. For example the y axis in the psmove
						api is the z axis in rviz (also it's inverted). Which is
						why the indecies set up so oddly.'''
						x=pos[2]
						y=pos[0]
						z=-pos[1]
						circle[key] = x,y,z
						print circle[key]
			else:
				'''if not tracking move will rumble'''
				move.set_rumble(120)

	# print circle
	true_x,true_y,true_z = circle['center']
	comps = []
	'''The z,y,z offset from the camera'''
	offset = [true_x,true_y,true_z]
	true_circle = {}
	'''Gets the coords for a circle around the user with the center being at the users chest'''
	for key in circle:
		true_circle[key] = [x - y for x, y in zip(circle.get(key), offset)]
	print true_circle

	'''Gets the radius of of x,y,z as it relates to the center of the user. Then compresses it
	based on the maximum reach of the robot'''
	radius = true_circle['forward'][0]
	comps.append(radius/max_reach)
	radius = true_circle['right'][1]
	comps.append(radius/max_reach)
	radius = true_circle['up'][2]
	comps.append(radius/max_reach)
	matrix = [offset,comps]
	'''returns the calibration matrix of how we need to shrink the values'''
	return matrix


rospy.init_node('psmove_broadcaster', anonymous=True)
follow = Follow('left', calibrate())
rospy.on_shutdown(follow.clean_shutdown)
follow.puppet()

'''
ISSUES AND EXPLAINATIONS: 
1) Fusion module was made for use with Open-GL based augmented reality applications.
So the position coordiates given are not with respect to any unit of measurement. Also
the values can be large and very 'bouncy' so the pos coordinates cannot simple be taken
raw and passed to the robot. Additionally 0,0,0 is located at the camera and not the user.
So moving the move away from you toward the camera would bring the arm closer to the robot
which is obviously a problem. 
To attempt to fix this issue the program will take coordinates from the center, forward, 
left and right of the user. To get a coordinate offset for where the user is located and
this is the users range of motion in x,y and z. That offset is then subtracted from the
raw position coords to get one: an origin that is no longer set at the camera it's now centered 
at the users chest and two: a radius from the center for x,y and z. Those radius's are then
divided by the max reach of the robot arm to get the ammount of compression that needs to be
applied to each axis to shrink the values to ensure they are within the robots reach. Then in
get_pose the math is basically: 

value_passed_to_robot = (raw_pos_from_move - offset_from_user)/compression

for each axis so that the robot will more or less mirror the users movements. 
The movements are not one to one and could use some improvement.

2) The position coordinates are based on the cameras field of view. So if you change the 
angle of the camera or your relationship to it in anyway you change how your position is 
interpreted (i.e. forward to you may not be forward to the robot depending on how the camera is 
looking at you). I have not been able to fix this.
'''