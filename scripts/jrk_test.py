#!/usr/bin/python

import rospy
#import sys
#import std_msgs
from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
import time


print("starting jrk_test")

#jrk_pub = rospy.Publisher('jrk_target', UInt16MultiArray, queue_size=1)
jrk_pub_raw = rospy.Publisher('jrk_target_raw', UInt16MultiArray, queue_size=1)

#INITIALIZE ROS NODE#
nodeName = "jrk_test"
logLevel = rospy.get_param('mode','DEBUG')
if logLevel == 'RUN':
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.WARNING, disable_signals=True)
elif logLevel == 'INFO':
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.INFO, disable_signals=True)
else:
	rospy.init_node(nodeName, anonymous=False, log_level=rospy.DEBUG, disable_signals=True)
rospy.loginfo("[Node Initialized] < running %s on %s mode >"%(nodeName,logLevel))
######################

arr = UInt16MultiArray()

def pub(L,R):
	arr.data = [L,R]
	#jrk_pub.publish(arr)
	jrk_pub_raw.publish(arr)

def stop():
	return pub(2048,2048)

def sweep():
	rospy.loginfo("> Sweep Started \n")
	for i in range(0,4095,10):
		rospy.logdebug("Sweep >> %d"%(i))
		time.sleep(0.005)
		pub(i,i)
	rospy.loginfo("> Sweep Finished")
	return stop()

def _execute(cmd):
	# print (cmd)
	if cmd == '':
		return
	if cmd == 'sweep':
		return sweep()
	if cmd[0] == 's':
		return stop()
	if cmd[0] == 'w':  #command shoud be like 'w 200 200' or 'write 200 200'
		a = cmd.split(' ')
		if len(a) == 2:
			a.append(a[1])
		rospy.loginfo('writing %s,%s'%(a[1],a[2])) 
		return pub((int(a[1])/100.0*2048+2048),(int(a[2])/100.0*2048+2048))
	if cmd[0] == 'r':
		a = cmd.split(' ')
		if len(a) == 2:
			a.append(a[1])
		rospy.loginfo('writing [raw] %s,%s'%(a[1],a[2]))
		return pub((int(a[1])),(int(a[2])))



rospy.loginfo ('>>> jrk test node initialized. \n\nAvailable commands: sweep, s, w, r \n\n')
while True:
	try:
		_execute(raw_input("jrk test cmd>>  "))
	except KeyboardInterrupt:
		rospy.logerr('Interrupted by User')
		break
	# except:
	# 	rospy.logerr('External Error')
	# 	break
