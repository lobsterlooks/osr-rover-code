#!/usr/bin/env python
import time
import rospy
from osr_msgs.msg import Status,Led
from screen import LedScreen

screen = LedScreen()

def callback(data):
	screen.build_msg(	
						int(rospy.get_param("remote_control/connected")),
						data.battery,
						data.error_status,
						data.temp,
						data.current,
						int(rospy.get_param("face/value"))
					)
	
	screen.check_for_afffirm()

def shutdown():
	screen.transistion_screen_to_idle()

if __name__ == "__main__":

	rospy.init_node("led_screen")
	rospy.loginfo("Starting the led_screen node")
	rospy.on_shutdown(shutdown)
	sub = rospy.Subscriber("/status", Status, callback)
	rospy.spin()
