#!/usr/bin/python

import rospy
#import std_msgs
from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray

print("starting jrk_test")

jrk_pub = rospy.Publisher('jrk_target', UInt16MultiArray, queue_size=1)
rospy.init_node('test_jrk')

arr = UInt16MultiArray()

def pub(L,R):
	arr.data = [L,R]
	jrk_pub.publish(arr)

def stop():
	return pub(2048,2048)

def sweep():
    for i in range(0,4095,10):
        print('Sweep>> %d'%i)
	pub(i,i)
        rospy.sleep(0.02)
    return stop()

def _execute(cmd):
	if cmd == 'sweep':
		return sweep()
	if cmd[0] == 's':
		return stop()
	if cmd[0] == 'w':  #command shoud be like 'w 200 200' or 'write 200 200'
		a = cmd.split(' ')
		if len(a) == 2:
			a.append(a[1])
		print ('writing %s,%s'%(a[1],a[2]))
		return pub((int(a[1])/100.0*2048+2048),(int(a[2])/100.0*2048+2048))
	if cmd[0] == 'r':
		a = cmd.split(' ')
		if len(a) == 2:
			a.append(a[1])
		print ('writing [raw] %s,%s'%(a[1],a[2]))
		return pub((int(a[1])),(int(a[2])))


print ('>>> jrk tester initialized. /n/n Available commands: sweep, s, w, r')
while True:
	try:
		_execute(raw_input("jrk test cmd>>  "))
	except rospy.ROSInterruptException:
		break
	except:
		print ('External Error')
		break
