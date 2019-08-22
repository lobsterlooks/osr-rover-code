#!/usr/bin/env python
import time
import rospy
from osr_msgs.msg import Commands, Encoder, Status
from roboclaw_wrapper import MotorControllers



def shutdown():
	cmds.drive_motor = [0] * 6
	pub.publish(cmds)


if __name__ == "__main__":

	rospy.init_node("test_driver")
	rospy.loginfo("Starting the test_driver node")
	rospy.on_shutdown(shutdown)
	pub = rospy.Publisher("/robot_cmds", Commands, queue_size =1)
	
	cmds = Commands()
	cmds.corner_motor = [-1] * 4
	
	rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
		try:
			text = int(raw_input("Enter Wheel speeds: "))
			drive_cmd = [text]*6
			print drive_cmd
		except:
			drive_cmd = [0] * 6
		cmds.drive_motor = drive_cmd
		pub.publish(cmds)
