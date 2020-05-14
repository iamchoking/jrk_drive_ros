#!/usr/bin/python

import rospy
#import std_msgs

from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Twist

from pynput import keyboard


ACCELERATION = 100  #% per second accerleration
DECELERATION = 0.95  #first-order decelerarion factor
ROTATION_ACC = 150   #% rotational acceleration for 

_left = 0
_right = 0
_trans = 0
_rot = 0
_state_left = False
_state_right = False
_state_up = False
_state_down = False

jrk_pub = rospy.Publisher('jrk_target_raw', UInt16MultiArray, queue_size=1)

#INITIALIZE ROS NODE#
nodeName = "jrk_teleop_key"
logLevel = rospy.get_param('mode','DEBUG')
if logLevel == 'RUN':
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.WARNING)
elif logLevel == 'INFO':
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.INFO)
else:
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.DEBUG)
rospy.loginfo("[Node Initialized] < running %s on %s mode >"%(nodeName,logLevel))
######################

arr = UInt16MultiArray()

def conv(a):
	if a >100:
		return conv(100)
	if a <-100:
		return conv(-100)
	return int(float(a)/100.0*2048+2048)

def pub(*v):
	global _left
	global _right

	if _left > 100:
		_left = 100
	if _right > 100:
		_right = 100
	if _left < -100:
		_left = -100
	if _right < -100:
		_right = -100

	rospy.logdebug('Teleop >> L: %.1f p., R: %.1f p., TRN: %.1f, ROT: %.1f'%(_left,_right,_trans,_rot))
	if len(v) == 0:
		arr.data = [conv(int(_left)),conv(int(_right))]
	elif len(v) == 1:
		arr.data = [v[0],v[0]]
	elif len(v) == 2:
		arr.data = [v[0],v[1]]
	jrk_pub.publish(arr)


def drive():
	global _left,_right,_state_left,_state_right,_state_up,_state_down,_trans,_rot
	_state_trans = (_state_up,_state_down)
	_state_rot =  (_state_left,_state_right)
	if _state_trans == (True,True):
		pass #cruise
	if _state_trans == (True,False):
		if _trans < 0:
			_trans = 0
		else:
			_trans += ACCELERATION/100.0
	if _state_trans == (False,True):
		if _trans > 0:
			_trans = 0
		else:
			_trans -= ACCELERATION/100.0
	if _state_trans == (False,False):
		if abs(_trans) < 2:
			_trans = 0
		else:
			_trans *= DECELERATION

	if _state_rot == (True,True):
		pass #cruise
	if _state_rot == (True,False):
		if _rot >0:
			_rot = 0
		else:
			_rot -= ROTATION_ACC/100.0
	if _state_rot == (False,True):
		if _rot <0:
			_rot = 0
		else:
			_rot += ROTATION_ACC/100.0
	if _state_rot == (False,False):
		if abs(_rot) < 2:
			_rot = 0
		else:
			_rot *= DECELERATION

	rospy.sleep(0.01)
	if _trans >= 100:
		_trans = 100
	if _trans <= -100:
		_trans = -100
	if _rot >= 100:
		_rot = 100
	if _rot <= -100:
		_rot = -100

	_left = _trans + _rot
	_right = _trans - _rot
	return pub()


def on_press(key):
	#print('{0} pressed'.format(key))
	global _left,_right,_state_left,_state_right,_state_up,_state_down
	if type(key) != type(keyboard.Key.esc):
		return drive()
	if key == key.up:
		_state_up = True
	if key == key.down:
		_state_down = True
	if key == key.left:
		_state_left = True
	if key == key.right:
		_state_right = True
	return drive()

def on_release(key):
	#print('{0} released'.format(key))
	global _left,_right,_state_left,_state_right,_state_up,_state_down
	if type(key) != type(keyboard.Key.esc):
		return drive()
	if key == key.up:
		_state_up = False
	if key == key.down:
		_state_down = False
	if key == key.left:
		_state_left = False
	if key == key.right:
		_state_right = False
	if key == key.esc:
		return False
	return drive()
rospy.loginfo(">>> jrk key teleop initialized. Press arrow keys for manuver, esc to exit")
with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
    listener.join()

