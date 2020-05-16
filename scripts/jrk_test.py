#!/usr/bin/python

import rospy
#import sys
import std_msgs
#from std_msgs.msg import String
#from std_msgs.msg import UInt16MultiArray
#import time


print("starting jrk_test")

#jrk_pub = rospy.Publisher('jrk_target', UInt16MultiArray, queue_size=1)
jrk_pub_raw = rospy.Publisher('jrk_target_raw', std_msgs.msg.UInt16MultiArray, queue_size=1)
jrk_pub_rpm = rospy.Publisher('jrk_target_rpm', std_msgs.msg.Float32MultiArray, queue_size=1)
jrk_pub_vel = rospy.Publisher('jrk_target_vel', std_msgs.msg.Float32MultiArray, queue_size=1)


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


def pub(L,R,mode = " "):
	if mode == ' ':
		arr = std_msgs.msg.UInt16MultiArray()
		arr.data = [L,R]
		#jrk_pub.publish(arr)
		jrk_pub_raw.publish(arr)
	elif mode == 'r': #rpm mode
		arr = std_msgs.msg.Float32MultiArray()
		arr.data = [L,R]
		jrk_pub_rpm.publish(arr)

	elif mode == 'v': #velocity mode
		arr = std_msgs.msg.Float32MultiArray()
		arr.data = [L,R]
		jrk_pub_vel.publish(arr)


def stop():
	return pub(2048,2048)

def sweep():
	rospy.loginfo("> Sweep Started")
	for i in range(0,4095,10):
		rospy.logdebug("Sweep >> %d"%(i))
		rospy.sleep(0.005)
		pub(i,i)
	rospy.sleep(0.1)
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
	
	if cmd[0] == 'w':
		a = cmd.split(' ')
		if len(a) == 2:
			a.append(a[1])

		if cmd[1] == ' ':  #command shoud be like 'w 200 200' or 'write 200 200'
			rospy.loginfo('writing [raw] %s,%s'%(a[1],a[2]))
			return pub((int(a[1])),(int(a[2])))
		if cmd[1] == 'p':
			rospy.loginfo('writing [raw] %sp., %sp.'%(a[1],a[2]))
			return pub((int(a[1])/100.0*2048+2048),(int(a[2])/100.0*2048+2048))
		elif cmd[1] == 'r':
			rospy.loginfo('writing [rpm] %s,%s'%(a[1],a[2]))
			return pub(float(a[1]),float(a[2]),'r')
		elif cmd[1] == 'v':
			rospy.loginfo('writing [vel] %s,%s'%(a[1],a[2]))
			return pub(float(a[1]),float(a[2]),'v')		



rospy.loginfo ('>>> jrk test node initialized. \n\nAvailable commands: sweep, s, w, wp, wr, wv \n\n')

rospy.sleep(1)
rospy.loginfo(">>> Starting Test Console \n\n")
while True:
	try:
		_execute(raw_input("jrk test cmd>>  "))
	except KeyboardInterrupt:
		rospy.loginfo('Interrupted by User')
		break
	rospy.sleep(0.2)
	# except:
	# 	rospy.logerr('External Error')
	# 	break
