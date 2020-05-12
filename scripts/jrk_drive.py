#!/usr/bin/python

import rospy
import std_msgs

import serial
import time

from geometry_msgs.msg import Twist

_Left =  '00287664'
_Right = '00291758'

#Connects jrk motor drivers with given serials
_Left_Operational = True
_Right_Operational = True

rospy.init_node("jrk_drive", anonymous=False)

# serL: /dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_18v19_00287664-if01
# serR: /dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_18v19_00291758-if01
rospy.loginfo('<<attepting Connection>> \n   >SerL: /dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_18v19_%s-if01\n   >SerR: /dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_18v19_%s-if01',_Left,_Right)

try:
	serL = serial.Serial( "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_18v19_%s-if01"%(_Left), 9600)
	rospy.loginfo(">Left Motor Successfully Connected!: Serial: %s",_Left)
except:
	rospy.logwarn("!>Left  Motor Connection Unsuccessful: Serial: %s",_Left)
	_Left_Operational = False
try:
	serR = serial.Serial( "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_18v19_%s-if01"%(_Right),9600)
	rospy.loginfo(">Right Motor Successfully Connected!: Serial: %s",_Right)
except:
	rospy.logwarn("!>Right Motor Connection Unsuccessful: Serial: %s",_Right)
	_Right_Operational = False
#ser = serial.Serial( "/dev/ttyACM0", 115200)
#print("connected to: " + serL.portstr)

if _Left_Operational or _Right_Operational:
	def _drive(ser,target,side):
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
		ser.write(chr(lowByte))
		ser.write(chr(highByte))
		rospy.loginfo("Drive>> %s, target: %s, Written: %d, %d" %(side,target,lowByte,highByte))
		#print(type(lowByte))


	def subscriber_Uint16MultiArray(message):
		rospy.loginfo("Sub>> detected incoming UInt16MultiArray : [ %s  %s ]"%(message.data[0],message.data[1]))
		if _Left_Operational:
			_drive(serL,message.data[0],"Left ")
		if _Right_Operational:
			_drive(serR,message.data[1],"Right")

	#rospy.init_node("jrk_drive", anonymous=False)
	rospy.Subscriber("jrk_target", std_msgs.msg.UInt16MultiArray, subscriber_Uint16MultiArray)
	rospy.loginfo('>>> Jrk Motor Driving Node Initialized <<<')
	rospy.spin()
else:
	rospy.logerr("No Motors Were Connected. Shutting Down.")
	rospy.signal_shutdown("No Motor Connection")


