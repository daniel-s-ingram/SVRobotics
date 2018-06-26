#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

rospy.init_node('joint_controller', anonymous=True)

joints_pub = rospy.Publisher('/ur10/position_cmd', Float32MultiArray, queue_size=10)
joints_msg = Float32MultiArray()
joints_msg.data = [0, 0, 0, 0, 0, 0]

def joint1_callback(msg):
	joints_msg.data[0] = -msg.data

def joint2_callback(msg):
	joints_msg.data[1] = -msg.data

def joint3_callback(msg):
	joints_msg.data[2] = -msg.data

rospy.Subscriber('/joint1/position/cmd', Float32, joint1_callback)
rospy.Subscriber('/joint2/position/cmd', Float32, joint2_callback)
rospy.Subscriber('/joint3/position/cmd', Float32, joint3_callback)

while not rospy.is_shutdown():
	joints_pub.publish(joints_msg)