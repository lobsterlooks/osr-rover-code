#!/usr/bin/env python
from roboclaw import Roboclaw
import time
import serial
import math
import rospy
import RPi.GPIO as GPIO

class MotorControllers(object):

	'''
	Motor class contains the methods necessary to send commands to the motor controllers

	for the corner and drive motors. There are many other ways of commanding the motors

	from the RoboClaw, we suggest trying to write your own Closed loop feedback method for

	the drive motors!

	'''
	def __init__(self):
		rospy.loginfo( "Initializing motor controllers")
		self.rc = Roboclaw( rospy.get_param('motor_controller/device/', "/dev/serial0"),
							rospy.get_param('motor_controller/baud', 115200))
		address_raw = rospy.get_param('motor_controller/addresses',"128,129,130,131,132")
		
		self.estop_pin        = 23
		self.accel            = [0]    * 10
		self.qpps             = [None] * 10
		self.err              = [None] * 5
		self.drive_zero_flag  = [0] * 6
		self.corner_zero_flag = [0] * 4
		address_list = (address_raw.split(','))
		self.address = [None]*len(address_list)
		
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.estop_pin,GPIO.OUT)
		self.set_estop_line(False)
		self.rc.Open()
		
		for i in range(len(address_list)):
			self.address[i] = int(address_list[i])

		version = [0] * 5
		comm_error =""
		for i in range(len(self.address)):
			print ("Attempting to talk to motor controller",self.address[i])
			version[i] = self.rc.ReadVersion(self.address[i])[0]
			if version[i] == 0:
				comm_error += " " + str(self.address[i])
		if (version != [0] * 5):
			print "[Motor__init__] Sucessfully connected to RoboClaw motor controllers"
		else:
			self.cleanup()
			raise Exception("Unable to establish connection to Roboclaw motor controllers" + comm_error)


		self.killMotors()
		self.enc_min =[]
		self.enc_max =[]
		
		for address in self.address:
			#self.rc.SetMainVoltages(address, rospy.get_param('battery_low', 11)*10), rospy.get_param('battery_high', 18)*10))

			if address == 131 or address == 132:
				while(self.rc.ReadM1PositionPID(address)[-1] == 0 or self.rc.ReadM2PositionPID(address)[-1] == 0):
					time.sleep(1)

				self.enc_min.append(self.rc.ReadM1PositionPID(address)[-2])
				self.enc_min.append(self.rc.ReadM2PositionPID(address)[-2])
				self.enc_max.append(self.rc.ReadM1PositionPID(address)[-1])
				self.enc_max.append(self.rc.ReadM2PositionPID(address)[-1])

			else:
				#self.rc.SetM1MaxCurrent(address, int(config['MOTOR_CONFIG']['max_drive_current']*100))
				#self.rc.SetM2MaxCurrent(address, int(config['MOTOR_CONFIG']['max_drive_current']*100))
				self.rc.ResetEncoders(address)


		rospy.set_param('motor_controller/enc_min', str(self.enc_min)[1:-1])
		rospy.set_param('motor_controller/enc_max', str(self.enc_max)[1:-1])
		
		for address in self.address:
			self.rc.WriteNVM(address)

		for address in self.address:
			self.rc.ReadNVM(address)
		'''
		voltage = self.rc.ReadMainBatteryVoltage(0x80)[1]/10.0
		if voltage >= rospy.get_param('low_voltage',11):
			print "[Motor__init__] Voltage is safe at: ",voltage, "V"
		else:
			raise Exception("Unsafe Voltage of" + voltage + " Volts")
		'''
		i = 0

		for address in self.address:
			self.qpps[i]    = self.rc.ReadM1VelocityPID(address)[4]
			self.accel[i]   = int(self.qpps[i]*2)
			self.qpps[i+1]  = self.rc.ReadM2VelocityPID(address)[4]
			self.accel[i+1] = int(self.qpps[i]*2)
			i+=2
		accel_max = 655359
		accel_rate = 0.5
		self.accel_pos = int((accel_max /2) + accel_max * accel_rate)
		self.accel_neg = int((accel_max /2) - accel_max * accel_rate)
		self.errorCheck()
		mids = [None]*4
		self.enc = [None]*4
		for i in range(4):
			mids[i] = (self.enc_max[i] + self.enc_min[i])/2
		self.cornerToPosition(mids)
		rospy.set_param("motor_controller/init", 1)
		time.sleep(2)
		self.killMotors()
		

	def cornerToPosition(self,tick):
		'''
		Method to send position commands to the corner motor

		:param list tick: A list of ticks for each of the corner motors to
		move to, if tick[i] is 0 it instead stops that motor from moving

		'''
		speed, accel = 1000,2000            #These values could potentially need tuning still
		for i in range(4):
			index = int(math.ceil((i+1)/2.0)+2)

			if tick[i] != -1:
				#print "Sending setpoint: ", tick[i]
				#rospy.loginfo(tick[i])
				self.corner_zero_flag[i] = 0
				if (i % 2):  x = self.rc.SpeedAccelDeccelPositionM2(self.address[index],accel,speed,accel,tick[i],1)
				else:        x = self.rc.SpeedAccelDeccelPositionM1(self.address[index],accel,speed,accel,tick[i],1)				
			elif self.corner_zero_flag[i] != 1:
				#print "Stopping corner motor: ", i
				self.corner_zero_flag[i] = 1
				if not (i % 2): x = self.rc.ForwardM1(self.address[index],0)
				else:           x = self.rc.ForwardM2(self.address[index],0)
			#else:
				#print "No command sent"

	def sendMotorDuty(self, motorID, speed):
		'''
		Wrapper method for an easier interface to control the drive motors,

		sends open-loop commands to the motors

		:param int motorID: number that corresponds to each physical motor
		:param int speed: Speed for each motor, range from 0-127

		'''
		#speed = speed/100.0
		#speed *= 0.5
		addr = self.address[int(motorID/2)]
		if speed > 0:
			if not motorID % 2: command = self.rc.ForwardM1
			else:               command = self.rc.ForwardM2
		else:
			if not motorID % 2: command = self.rc.BackwardM1
			else:               command = self.rc.BackwardM2

		speed = abs(int(speed * 127))

		return command(addr,speed)

	def sendSignedDutyAccel(self,motorID,speed):
		addr = self.address[int(motorID/2)]

		if speed >0: accel = self.accel_pos
		else: accel = self.accel_neg

		if not motorID % 2: command = self.rc.DutyAccelM1
		else:               command = self.rc.DutyAccelM2

		speed = int(32767 * speed/100.0)
		return command(addr,accel,speed)

	def getCornerEnc(self):
		enc = []
		for i in range(4):
			index = int(math.ceil((i+1)/2.0)+2)
			if not (i % 2):
				enc.append(self.rc.ReadEncM1(self.address[index])[1])
			else:
				enc.append(self.rc.ReadEncM2(self.address[index])[1])
		self.enc = enc
		return enc

	def temp(self,speed):
		for i in range(len(speed)):
			if i > 0:
				return
			
			addr = self.address[int(i/2)]
			
			if speed[i] >0: accel = self.accel_pos
			else: accel = self.accel_neg
			
			if(speed[i] == 0 and self.drive_zero_flag[i] == 0):
				self.drive_zero_flag[i] = 1
				if not i % 2:
					self.rc.ForwardM1(addr,0)
				else:
					self.rc.ForwardM2(addr,0)
			elif speed[i] != 0:
				speed_cmd = int(32767 * speed[i]/100.0)
				if not i % 2: 	
					self.rc.DutyAccelM1(addr,accel,speed_cmd)
				else:			
					self.rc.DutyAccelM2(addr,accel,speed_cmd)
	def temp2(self,speed):
		for i in range(len(speed)):
			addr = self.address[int(i/2)]
			
			if(speed[i] == 0 and self.drive_zero_flag[i] == 0):
				self.drive_zero_flag[i] = 1
				if not i % 2:
					self.rc.ForwardM1(addr,0)
				else:
					self.rc.ForwardM2(addr,0)
			elif speed[i] != 0:
				self.drive_zero_flag[i] = 0
				speed_cmd = abs(int(speed[i] * 127/100))
				
				if speed[i] > 0:
					if not i % 2:
						self.rc.ForwardM1(addr,speed_cmd)
					else:
						self.rc.ForwardM2(addr,speed_cmd)
				else:
					if not i % 2:
						self.rc.BackwardM1(addr,speed_cmd)
					else:
						self.rc.BackwardM2(addr,speed_cmd)
	@staticmethod
	def tick2deg(tick,e_min,e_max):
		'''
		Converts a tick to physical degrees

		:param int tick : Current encoder tick
		:param int e_min: The minimum encoder value based on physical stop
		:param int e_max: The maximum encoder value based on physical stop

		'''
		return (tick - (e_max + e_min)/2.0) * (90.0/(e_max - e_min))

	def getCornerEncAngle(self):
		if self.enc[0] == None:
			return -1
		deg = [None] *4

		for i in range(4):
			deg[i] = int(self.tick2deg(self.enc[i],self.enc_min[i],self.enc_max[i]))

		return deg

	def getDriveEnc(self):
		enc = [None]*6
		for i in range(6):
			if not (i % 2):
				enc[i] = self.rc.ReadEncM1(self.address[int(math.ceil(i/2))])[1]
			else:
				enc[i] = self.rc.ReadEncM2(self.address[int(math.ceil(i/2))])[1]
		return enc

	def getBattery(self):
		return self.rc.ReadMainBatteryVoltage(self.address[0])[1]
		
	def getTemp(self):
		temp = [None] * 5
		for i in range(5):
			temp[i] = self.rc.ReadTemp(self.address[i])[1]
		return temp
	
	def getCurrents(self):
		currents = [None] * 10
		for i in range(5):
			currs = self.rc.ReadCurrents(self.address[i])
			currents[2*i] = currs[1]
			currents[(2*i) + 1] = currs[2]
		return currents

	def getErrors(self):
		return self.err

	def killMotors(self):
		'''
		Stops all motors on Rover
		'''
		for i in range(5):
			self.rc.ForwardM1(self.address[i],0)
			self.rc.ForwardM2(self.address[i],0)

	def errorCheck(self):
		'''
		Checks error status of each motor controller, returns 0 if any errors occur
		'''

		for i in range(len(self.address)):
			self.err[i] = self.rc.ReadError(self.address[i])[1]
		for error in self.err:
			if error:
				self.killMotors()
				#self.writeError()
				rospy.loginfo("Motor controller Error", error)
		return 1

	def writeError(self):
		'''
		Writes the list of errors to a text file for later examination
		'''

		f = open('errorLog.txt','a')
		errors = ','.join(str(e) for e in self.err)
		f.write('\n' + 'Errors: ' + '[' + errors + ']' + ' at: ' + str(datetime.datetime.now()))
		f.close()

	def set_estop_line(self,val):
		'''
		'''
		print "Setting pins to ", str(not val)
		GPIO.output(self.estop_pin,not val)

	def cleanup(self):
		self.killMotors()
		GPIO.output(self.estop_pin,0)
		time.sleep(0.05)
		GPIO.cleanup()

