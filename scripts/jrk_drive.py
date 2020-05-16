#!/usr/bin/python

import rospy
import std_msgs

import serial
#import time

#from geometry_msgs.msg import Twist

import os 

DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class Jrk_Map:
	def __init__(self,VER = "G2_18v19",MAC = 'BAXTER'):
		#print dir_path
		self.ver = VER
		self.mac = MAC
		self.name = "%s-%s"%(MAC,VER)
		rospy.logdebug ("Jrk_Map >> initializing map for: %s"%(self.name))
		self.calibDir = os.path.join(DIR_PATH,"jrk_calibration_data","%s-calib.csv" %(self.name))
		self.mapDir   = os.path.join(DIR_PATH,"jrk_map_data"        ,"%s-map.csv"   %(self.name))

		try:
			rospy.logdebug("Jrk_Map >> Looking For: %s"%(self.calibDir))
			calib_raw = open(self.calibDir,'r').read().split('\n')
		except IOError:
			rospy.logdebug("Jrk_Map >> No Calibration Data Found!")
			return
		# print calib_raw[1:10],calib_raw[12:] #[1:10] sould be property values, [12:] should be target-calibration data
		
		# creates properties. it has radius, circumference, output_min, output_max, deadzone_low, deadzone_high, center, target_max, target_min
		self.properties = dict()
		calibIndex = 0
		for i in range(1,len(calib_raw)):
			if calib_raw[i][0] == "#":
				calibIndex = i+2
				break
			temp = calib_raw[i].split(',')
			if "output" in temp[0].lower() or "target" in temp[0].lower() or "center" in temp[0].lower():
				self.properties.update({temp[0] : int(temp[1])})
			else:
				self.properties.update({temp[0] : float(temp[1])})

		# print self.properties
		# print self.properties['radius']
		calib_pharse = []
		for i in calib_raw[calibIndex:]:
			temp = i.split(',')
			calib_pharse.append((int(temp[0]),float(temp[1]),self.properties["circumference"]*float(temp[1])/60))
		
		self.properties.update({
			"rpm_min": calib_pharse[ 0][1],
			"rpm_max": calib_pharse[-1][1],
			"vel_min": calib_pharse[ 0][2],
			"vel_max": calib_pharse[-1][2],
			})
		for i in range(len(calib_pharse)):
			if calib_pharse[i][0] == self.properties["deadzone_low"]:
				self.properties.update({
					"vel_dead_low": calib_pharse[i-1][2],
					"rpm_dead_low": calib_pharse[i-1][1]
				})
			if calib_pharse[i][0] == self.properties["deadzone_high"]:
				self.properties.update({
					"vel_dead_high": calib_pharse[i+1][2],
					"rpm_dead_high": calib_pharse[i+1][1]
				})

		# print self.properties
		# for i in calib_pharse:
		# 	print i

		index_cur = 0

		self.raw_map = [[0.0,0.0] for i in range(self.properties["output_min"],self.properties["output_max"])] #raw_map[x][0] rpm on target x , --[1] vel on target x
		for i in range(len(self.raw_map)):
			if i<self.properties["target_min"]:
				self.raw_map[i][0],self.raw_map[i][1] = self.properties["rpm_min"],self.properties["vel_min"]
				continue
			if i>self.properties["target_max"]:
				self.raw_map[i][0],self.raw_map[i][1] = self.properties["rpm_max"],self.properties["vel_max"]
				continue
			if i>calib_pharse[index_cur+1][0]:
				index_cur += 1
			
			self.raw_map[i][0],self.raw_map[i][1] = Jrk_Map.interpolate(i,calib_pharse[index_cur][0],calib_pharse[index_cur+1][0],calib_pharse[index_cur][1],calib_pharse[index_cur+1][1]),Jrk_Map.interpolate(i,calib_pharse[index_cur][0],calib_pharse[index_cur+1][0],calib_pharse[index_cur][2],calib_pharse[index_cur+1][2])
			#print "currently computing %d, between %d and %d"%(i,calib_pharse[index_cur][0],calib_pharse[index_cur+1][0])
			# if i%10 == 0:
			# 	print "computed %d : %f, %f interpolated from %s,%s" %(i,self.raw_map[i][0],self.raw_map[i][1],str(calib_pharse[index_cur]),str(calib_pharse[index_cur+1]))
		rospy.logdebug("Jrk_Map >> <%s> initialization complete"%(self.name))

	@staticmethod
	def interpolate(inp,inpL,inpH,outpL,outpH): #inpL -> outpL, inpH -> outpH, inp -> ?
		if inpH == inpL:
			return (outpL+outpH)/2
		
		if inp == inpH:
			return outpH
		elif inp == inpL:
			return outpL

		return outpL + (outpH-outpL) * ( (float(inp)-inpL) / (inpH-inpL) )

	def searchRpm(self,rpm):
		low = self.properties["output_min"]
		high = self.properties["output_max"]
		searches = 0
		if rpm >= self.properties["rpm_max"]:
			rospy.logwarn("Jrk_Map >> Required RPM exceeds capability. Required: %f, Capable: %f"%(rpm,self.properties["rpm_max"]))
			return self.properties["output_max"]
		elif rpm <= self.properties["rpm_min"]:
			rospy.logwarn("Jrk_Map >> Required RPM exceeds capability, Required: %f, Capable: %f"%(rpm,self.properties["rpm_max"]))
			return self.properties["output_min"]
		while True:
			target = (high+low)/2
			if self.raw_map[target][0] > rpm:
				high = target
			elif self.raw_map[target+1][0] < rpm:
				low = target
			else:
				break
			searches += 1
		#print searches
		return target

	def searchVel(self,vel): #binary searches the map for closest 
		low = self.properties["output_min"]
		high = self.properties["output_max"]
		searches = 0
		if vel >= self.properties["vel_max"]:
			rospy.logwarn("Jrk_Map >> Required Velocity exceeds capability. Required: %f, Capable: %f"%(vel,self.properties["vel_max"]))
			return self.properties["output_max"]
		elif vel <= self.properties["vel_min"]:
			rospy.logwarn("Jrk_Map >> Required Velocity exceeds capability. Required: %f, Capable: %f"%(vel,self.properties["vel_min"]))
			return self.properties["output_min"]
		while True:
			target = (high+low)/2
			if self.raw_map[target][1] > vel:
				high = target
			elif self.raw_map[target+1][1] < vel:
				low = target
			else:
				break
			searches += 1
		#print searches
		return target

class Jrk_Motor(Jrk_Map):
	def __init__(self,ser,side,ver = 'G2_18v19',mac = "BAXTER"):
		self.fullname = "%s-%s-%s"%(mac,ver,side)
		#print self.fullname
		try:
			Jrk_Map.__init__(self,ver,mac)
			self.is_map = True
		except:
			rospy.logwarn("Jrk_Motor >> map file creation for %s has failed"%(self.fullname))
			self.is_map = False
		rospy.loginfo("Initializing Motor [%s] Name [%s] with Serial [%s]"%(ver,self.fullname,ser))
		self.ver = ver
		self.serNo = ser
		self.side = side
		try:
			self.ser = serial.Serial( "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_%s_%s-if01"%(ver,ser), 9600)
			self.is_operational = True
			rospy.loginfo("Jrk_Motor >>  >Motor [%s] Name [%s] Serial [%s]: Connection Successful"%(ver,self.fullname,ser))
		except:
			self.ser = None
			self.is_operational = False
			rospy.logwarn("Jrk_Motor >> !>Motor [%s] Name [%s] Serial [%s]: Connection Unsuccessful"%(ver,self.fullname,ser))

	def drive(self,target):
		if not self.is_operational:
			rospy.logwarn("Jrk_Motor >> !! Motor %s is not operational and will not be driven !!"%(self.fullname))
			return
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
		rospy.logdebug("Jrk_Motor >> Drive>> %s, target: %s, Written: %d, %d" %(self.fullname,target,lowByte,highByte))

	def stop(self):
		self.drive(2048)
	
	def driveRpm(self,rpm):
		if not self.is_map:
			rospy.logerr("Jrk_Motor >> ! <%s> Rpm based drive requested. Detected no Map data !"%(self.fullname))
		self.drive(self.searchRpm(rpm))

	def driveVel(self,vel):
		if not self.is_map:
			rospy.logerr("Jrk_Motor >> ! <%s> Velocity based drive requested. Detected no Map data !"%(self.fullname))
		self.drive(self.searchVel(vel))


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
VERSION = 'G2_18v19'	#Polulu Device Version
MACHINE = "BAXTER"	#Machine Name

jrk_left  = Jrk_Motor(SERIAL_LEFT,"Left",VERSION,MACHINE)
jrk_right = Jrk_Motor(SERIAL_RIGHT,'Right',VERSION,MACHINE)

if jrk_left.is_operational or jrk_right.is_operational:
	def subscriber_raw(message):
		rospy.logdebug("SubRaw  >> detected incoming raw target [UInt16MultiArray]: [ %s  %s ]"%(message.data[0],message.data[1]))
		jrk_left.drive(message.data[0])
		jrk_right.drive(message.data[1])
		return
    
	def subscriber_rpm(message):
		rospy.logdebug("SubRPM  >> detected incoming raw target [float32MultiArray]: [ %s  %s ]"%(message.data[0],message.data[1]))
		jrk_left.driveRpm(message.data[0])
		jrk_right.driveRpm(message.data[1])
		return

	def subscriber_vel(message):
		rospy.logdebug("SubVel  >> detected incoming raw target [float32MultiArray]: [ %s  %s ]"%(message.data[0],message.data[1]))
		jrk_left.driveVel(message.data[0])
		jrk_right.driveVel(message.data[1])
		return


	rospy.Subscriber("jrk_target_raw", std_msgs.msg.UInt16MultiArray , subscriber_raw)
	rospy.Subscriber("jrk_target_rpm", std_msgs.msg.Float32MultiArray, subscriber_rpm)
	rospy.Subscriber("jrk_target_vel", std_msgs.msg.Float32MultiArray, subscriber_vel)
	rospy.loginfo('>>> Jrk Motor Driving Node Initialized <<<')
	rospy.spin()
	rospy.loginfo("[Shutdown] MOTORS STOPPED")
	jrk_left.stop()
	jrk_right.stop()
else:
	rospy.logerr("No Motors Were Connected. Shutting Down.")
	rospy.signal_shutdown("No Motors Connected")