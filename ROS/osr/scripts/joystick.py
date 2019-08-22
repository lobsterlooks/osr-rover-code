#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from osr_msgs.msg import Joystick,Led
from std_msgs.msg import Int64MultiArray
import math

global mode
global last
global counter
mode,counter = 0,0
last = time.time()

def callback(data):
	global mode
	global counter
	global last

	led_msg = Int64MultiArray()
	joy_out = Joystick()

	y =  data.axes[1]
	x = -data.axes[0]
	x1 =-data.axes[3]
	rt = data.axes[2]

	cmd = two_joy(x1,y,rt)
	
	dpad = data.buttons[11:]
	if 1 in dpad: mode = dpad.index(1)
	now = time.time()

	led_msg.data = [mode,1]
	if now - last > 0.75:
		counter +=1
	else:
		counter = 0
	if counter > 3:
		rospy.set_param("remote_control/connected",0)
	else:
		rospy.set_param("remote_control/connected",1)
	rospy.set_param("face/value",mode)
	last = time.time()
	led_pub.publish(led_msg)
	
	
	cmd = two_joy(x1,y,rt)
	joy_out = Joystick()
	joy_out.vel = cmd[0]
	joy_out.steering = cmd[1]
	joy_out.mode = mode
	
	if data.buttons[1]:
		rospy.loginfo("kill button")
		joy_out.steering = 0
		joy_out.vel = 0
	rospy.loginfo(joy_out)
	joy_out.connected = True
	pub.publish(joy_out)

def two_joy(x,y,rt):
	boost = 0
	if rt <= 0:
		boost = 50*-rt
	if y >=0: y = (y * 50) + boost
	else: y = (y *50) - boost
	x *= 100
	return (int(y),int(x))

if __name__ == '__main__':
	global pub
	#global led_pub
	rospy.init_node('joystick')
	rospy.loginfo('joystick started')

	sub = rospy.Subscriber("/joy", Joy, callback)
	pub = rospy.Publisher('joystick', Joystick, queue_size=1)
	led_pub = rospy.Publisher('led_cmds', Int64MultiArray, queue_size=1)

	rospy.spin()
