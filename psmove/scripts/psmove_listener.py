#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
    
def listener():

    rospy.init_node('psmove_listener', anonymous=True)

    rospy.Subscriber("/move_info_for_test", Float32MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
