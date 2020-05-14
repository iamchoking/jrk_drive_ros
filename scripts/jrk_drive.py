#!/usr/bin/python

import rospy
import std_msgs

import serial
import time

from geometry_msgs.msg import Twist

class Jrk_Motor:
	def __init__(self,ser,name,ver = 'G2_18v19'):
		rospy.loginfo("Initializing Motor [%s] Name [%s] with Serial [%s]"%(ver,name,ser))
		self.ver = ver
		self.serNo = ser
		self.name = name
		try:
			self.ser = serial.Serial( "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_%s_%s-if01"%(ver,ser), 9600)
			self.is_operational = True
			rospy.loginfo(" >Motor [%s] Name [%s] Serial [%s]: Connection Successful"%(ver,name,ser))
		except:
			self.ser = None
			self.is_operational = False
			rospy.logwarn("!>Motor [%s] Name [%s] Serial [%s]: Connection Unsuccessful"%(ver,name,ser))

	def drive(self,target):
		if target < 5:
			lowByte = 0	#ser.write(chr(0))
			highByte = 192	#ser.write(chr(192))
		elif target > 4091:
			lowByte = 218	#ser.write(chr(203))
			highByte = 127	#ser.write(chr(127))
		else:	
			lowByte = (target & ord("\x1F")) | ord("\xC0")
			highByte = (target >> 5) & ord("\x7F")
			#print("about to write", lowByte, highByte)
		self.ser.write(chr(lowByte))
		self.ser.write(chr(highByte))
		rospy.logdebug("Drive>> %s, target: %s, Written: %d, %d" %(self.name,target,lowByte,highByte))

	def stop(self):
		self.drive(2048)


#INITIALIZE ROS NODE#
nodeName = "jrk_drive"
logLevel = rospy.get_param('mode','DEBUG')
if logLevel == 'RUN':
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.WARNING)
elif logLevel == 'INFO':
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.INFO)
else:
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.DEBUG)
rospy.loginfo("[Node Initialized] < running %s on %s mode >"%(nodeName,logLevel))
######################

#JRK MOTOR CONNECTION SEQUENCE#
SERIAL_LEFT =  '00287664'
SERIAL_RIGHT = '00291758'

jrk_left  = Jrk_Motor(SERIAL_LEFT,"Left")
jrk_right = Jrk_Motor(SERIAL_RIGHT,'Right')

if jrk_left.is_operational or jrk_right.is_operational:
	def subscriber_raw(message):
		rospy.logdebug("SubRaw  >> detected incoming raw target [UInt16MultiArray]: [ %s  %s ]"%(message.data[0],message.data[1]))
		if jrk_left.is_operational:
			jrk_left.drive(message.data[0])
		if jrk_right.is_operational:
			jrk_right.drive(message.data[1])
		return
    
	rospy.Subscriber("jrk_target_raw", std_msgs.msg.UInt16MultiArray, subscriber_raw)
	rospy.loginfo('>>> Jrk Motor Driving Node Initialized <<<')
	rospy.spin()
	rospy.loginfo("[Shutdown] MOTORS STOPPED")
	jrk_left.stop()
	jrk_right.stop()
else:
	rospy.logerr("No Motors Were Connected. Shutting Down.")
	rospy.signal_shutdown("No Motors Connected")