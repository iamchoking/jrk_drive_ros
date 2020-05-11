#!/usr/bin/python

import rospy
#import std_msgs

from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray

from mavros_msgs.msg import RCIn


DECELERATION = 0.9  #first-order decelerarion factor
ROTATION_ACC = 150   #% rotational acceleration for 

_left = 0
_right = 0
_trans = 0
_rot = 0

jrk_pub = rospy.Publisher('jrk_target', UInt16MultiArray, queue_size=1)
rospy.init_node('jrk_mavlink_bridge')

arr = UInt16MultiArray()


######################## MESSAGE TRANSLATION (Recieved) ########################

def interpolate(a,b,x,yl,yh): #x = a returns yl, x = b returns yh
	return yl+(abs(yl-yh)*1.0/(b-a))*(x-a)

def percent(rule,value,*deadzone):
	#rule= (low,mid,high,signed), or (low, high,signed), deadzone indicates the raw error to be accepted zero
	if len(deadzone) == 0:
		dead = 0
	if len(deadzone) != 0:
		dead = deadzone[0]
	if len(rule) == 4:
		low,mid,high,signed = rule
		if signed:
			if (value - mid)**2<dead**2:
				return 0
			elif value<=mid:
				return interpolate(low,mid-dead,value,-100,0)
			elif value>=mid:
				return interpolate(mid+dead,high,value,0,100)
		else:
			if (value - mid)**2<dead**2:
				return 50
			elif value<=mid:
				return interpolate(low,mid-dead,value,0,50)
			elif value>=mid:
				return interpolate(mid+dead,high,value,50,100)
	if len(rule) == 3:
		low,high,signed = rule
		if signed:
			return interpolate(low,high,value,-100,100)
		else:
			return interpolate(low,high,value,0,100)


def translate(channel): #left,back,down is -negative
#channels Descriptions - DJI Mode 2 controller SBUS
#0: roll (right horiz) ((rolling left (full left))995 - 1494 - 1993(rolling right(full right)))
#1: pitch (right vert)  (984(pitching up(full back)) - 1488 - 2005(pitching down(full front)))
#2: throttle (left vert) (993 - 2003)
#3: yaw (left horiz) (993(heading left(full left)) - 1490 - 1989 (heading right(full right)))
#4: SA lever (994(down) 1494(middle) 1994(up))
#5: SB lever (994(front) 1494(middle) 1994(back))
#6: SC lever (994(front) 1494(middle) 1994(back))
#7: SD lever (994(down) 1494(middle) 1994(up))
#(there are a total of 18 channels (up to #17) but #8 and above do not seem to change with buttons)
	if len(channel) == 0:
		print "Mavlink Signal >> No Signal"
		return (0,0,0,0,0,0,0,0)
	roll = percent((995,1494,1993,True),channel[0],3)
	pitch = percent((984,1488,2005,True),channel[1],3)
	yaw = percent((993,1490,1989,True),channel[3],3)
	throttle = percent((993,2003,False),channel[2],3)
	if channel[4] == 994:
		SA = -1
	elif channel [4] == 1494:
		SA = 0
	else:
		SA = 1

	if channel[5] == 994:
		SB = 1
	elif channel [5] == 1494:
		SB = 0
	else:
		SB = -1

	if channel[6] == 994:
		SC = 1
	elif channel [6] == 1494:
		SC = 0
	else:
		SC = -1

	if channel[7] == 994:
		SD = -1
	elif channel [7] == 1494:
		SD = 0
	else:
		SD = 1
	print 'Mavlink Signal >> R: %.2f, P: %.2f, Y: %.2f, T: %.2f, SA = %d,SB = %d, SC = %d, SD = %d' %(roll,pitch,yaw,throttle,SA,SB,SC,SD)
	return roll,pitch,yaw,throttle,SA,SB,SC,SD

######################## MESSAGE TRANSLATION (To send) ########################

def conv(a):
	if a >100:
		return conv(100)
	if a <-100:
		return conv(-100)
	return int(float(a)/100.0*2048+2048)

def pub(*v):
	global _left,_right,_trans,_rot

	if _trans >= 100:
		_trans = 100
	if _trans <= -100:
		_trans = -100

	if _rot >= 100:
		_rot = 100
	if _rot <= -100:
		_rot = -100

	_left = _trans + _rot
	_right= _trans - _rot

	if _left > 100:
		_left = 100
	if _right > 100:
		_right = 100
	if _left < -100:
		_left = -100
	if _right < -100:
		_right = -100

	print ('Mavlink Teleop >> L: %.1f p., R: %.1f p., TRN: %.1f, ROT: %.1f'%(_left,_right,_trans,_rot))
	if len(v) == 0:
		arr.data = [conv(int(_left)),conv(int(_right))]
	elif len(v) == 1:
		arr.data = [v[0],v[0]]
	elif len(v) == 2:
		arr.data = [v[0],v[1]]
	jrk_pub.publish(arr)

######################## Driving Code ########################
#drivemode is toggled by SD (-1: OFF, 0: mode 1, 1: mode 2)

def mode1(M):
#'car' mode: speed controlled by throttle, steer controlled by roll, SC is gear switch: (1(front): D, 0(middle): N, -1(back): R)
	global _left,_right,_trans,_rot
	roll,pitch,yaw,throttle,SA,SB,SC,SD = M
	if SC == 0:
		if abs(_rot) < 2:
			_rot = 0
		else:
			_rot *= DECELERATION
		if abs(_trans) < 2:
			_trans = 0
		else:
			_trans *= DECELERATION
	elif SC == 1:
		_trans = throttle
		_rot = roll
	elif SC == -1:
		_trans = -throttle
		_rot = roll
	return pub()

def mode2(M):
#'joystick' mode: speed controlled by pitch, steer controlled by roll (no gear switch)
	global _left,_right,_trans,_rot
	roll,pitch,yaw,throttle,SA,SB,SC,SD = M
	_trans = pitch
	_rot = roll
	return pub()

def op (message):
	M = translate(message.channels)
	if M[-1] == -1:
		return pub(2048,2048)
	if M[-1] == 0:
		return mode1(M)
	if M[-1] == 1:
		return mode2(M)



rospy.Subscriber('/mavros/rc/in',RCIn, op)
rospy.spin()

