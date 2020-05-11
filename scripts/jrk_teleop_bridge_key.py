#!/usr/bin/python

import rospy
#import std_msgs

from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Twist

from pynput import keyboard

#pynput example code
'''
def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
	print type(key.char)
    except AttributeError:
        print('special key {0} pressed'.format(key))
	print type(key)
	print key == key.up

def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener on esc key release
        return False

# Collect events until released
with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
    listener.join()
'''

jrk_pub = rospy.Publisher('jrk_target', UInt16MultiArray, queue_size=1)
rospy.init_node('jrk_teleop_bridge')


_left = 0
_right = 0

arr = UInt16MultiArray()
def conv(a):
	return int(float(a)/100.0*2048+2048)
def pub(L,R):
	arr.data = [L,R]
	jrk_pub.publish(arr)
def conversion(message):
	print ('bridging Twist:[lin %.2f ang %.2f] to UIntMultiArray:[Left %d perc. Right %d perc.]' %(message.linear.x, message.angular.z,message.linear.x*100,message.linear.x*100))
	pub(conv(message.linear.x*100),conv(message.linear.x*100))

rospy.Subscriber('key_vel', Twist, conversion)
rospy.spin()
