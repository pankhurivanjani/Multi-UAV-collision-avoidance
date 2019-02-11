#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from sensor_msgs.msg import NavSatFix 
#from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State 
from mavros_msgs.srv import *

#Initializing with opposite values
connected = False
armed = False
mode = "GUIDED" 

#Global pos variables initialization
latitude =0.0
longitude=0.0
altitude =0.0
#Global velocity variables initialization
vel_x = 0.0
vel_y = 0.0
vel_z = 0.0
#Global stable flight start time object
stable_start_time = rospy.Time()
#Flag for tasks
task_1 = False
task_2 = False
#Velocity setting variable
v = TwistStamped()
#Global pos variables initialization for master
latitude_m =0.0
longitude_m=0.0
altitude_m =0.0

#Current state of vehicle
def state_cb(msg):
	global connected
	global armed
	global mode
	connected = msg.connected
	armed = msg.armed
	mode = msg.mode	

#GPS position fix reported by the device.
def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    global altitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    altitude = globalPositionCallback.altitude

def globalPositionCallback_master(globalPositionCallback):
    global latitude_m
    global longitude_m
    global altitude_m
    latitude_m  = globalPositionCallback.latitude
    longitude_m = globalPositionCallback.longitude
    altitude_m  = globalPositionCallback.altitude

#Global Velocity fused by FCU
def localVel(val):
    global vel_x 
    global vel_y
    global vel_z
    vel_x = val.twist.linear.x
    vel_y = val.twist.linear.y
    vel_z = val.twist.linear.z

#Giving velocity commands
def velocityChange(pub, val):
	global v
	v.twist.linear.z = val
	pub.publish(v)
	rospy.loginfo("Entering data into log = " + str(v.twist.linear.z))

#Disable pos signal after
def trigger(home_alt, task, time, target):		
	global stable_start_time

	rospy.loginfo('Duration = '+ str((rospy.Time.now() - stable_start_time).secs) +'Height = '+ str(altitude - home_alt))	

	#recording stable flight time	 
	if(altitude - home_alt >= target or task ):
		if(rospy.Time.now() - stable_start_time > rospy.Duration(time) or task):#complete 10 sec stable flight at 2 m
			rospy.loginfo('*****Task Completed*******'+' Duration = '+str((rospy.Time.now() - stable_start_time).secs) +' Home_alt = '+ str(home_alt))
			task = True	
			return False
		else:
			return True #keep publishing
	else:		
		duration = rospy.Time() #set to time = 0 if unstable from pos 
		stable_start_time = rospy.Time.now()			
		return True 		#Task not achieved

#P-Controller for maintaining constant distance
def pControl(coeff, dist_val):
	global altitude
	global altitude_m
	global vel_z

	dist_curr = abs(altitude_m - altitude)
	rospy.loginfo('Current distance between master and support = ' + str(dist_curr))

	if altitude_m < altitude: #completing equation 
		coeff = -coeff
	
	#u = vel_z + coeff * (dist_curr - dist_val)
	u = coeff * (dist_curr - dist_val)	
	return u

#Main program-----------------------------------
if __name__ == '__main__':
	rospy.init_node('offboard_node', anonymous=True)

	rospy.Subscriber('/uav1/mavros/state', State, state_cb)  #State - connection/arming/mode info

	local_pos_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size = 10) #To give position

	set_mode_client = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)	#Setting Mode

	arming_client = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)	#Arming

	rospy.Subscriber("/uav1/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback) #Getting global gps pos

	rospy.Subscriber("uav1/mavros/local_position/velocity", TwistStamped, localVel) #local velocity show more precise values	

	pub = rospy.Publisher('uav1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 20) #To publish velocity commands

	rospy.Subscriber("/uav2/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback_master) #Master global gps pos

	#Setting rate for repetition / sleep time
	rate = rospy.Rate(20)	
		
	#Verifying connection between MAVlink and autopilot
	try:
		while((not rospy.is_shutdown()) and (not connected)): 
			print('---Waiting for connection---')
			rate.sleep()        
	except rospy.ROSInterruptException:
		pass	

	rospy.loginfo('Connection established')
	
	#Setting next coordinates
	pose = PoseStamped()
	pose.pose.position.x = 0
	pose.pose.position.y = 0
	pose.pose.position.z = 2

	#Continuously publishing coordinates before flight
	try:
		for i in range(1,101):
			local_pos_pub.publish(pose)
			print('Publishing position coordinates')
			rate.sleep()        
	except rospy.ROSInterruptException:
		pass	

	last_request = rospy.Time.now() #Taking a record of time

	a = True
	k = 0.3
	dist = 4

	#Main loop--------------------------
	while not rospy.is_shutdown():
		if (mode != "OFFBOARD") and (rospy.Time.now() - last_request > rospy.Duration(5.0)): #Set mode to OFFBOARD if not
			print('Setting mode')
			if set_mode_client(custom_mode = "OFFBOARD"):
				rospy.loginfo('OFFBOARD enabled')
			last_request = rospy.Time.now()
		else:
			if (not armed) and (rospy.Time.now() - last_request > rospy.Duration(5.0)): #ARM the vehicle if not 
				print('Arming vehicle')
				if arming_client(True).success:
					rospy.loginfo('Vehicle armed')
			#last_request = rospy.Time.now()
				
		#Record of home altitude
		
		if(a):
			home_alt = altitude
			a = False
		
		#publishing pos in loop for continuously maintaining pos
		if(trigger(home_alt, task_1, 10.0, 1.8)):
			local_pos_pub.publish(pose)
			print('POS PUBLISHED  Z = 2')
		else:
			task_1 = True
		#	if(not (altitude - home_alt > 2.0)):
			val = pControl(k, dist)
			velocityChange(pub, val)
			rospy.loginfo('------k ='+str(k)+ ' Const Distance = '+str(dist)+' Velocity Z = '+ str(val) +'----------')
		
		
		#printing global position
		rospy.loginfo(' Lon = '+ str(longitude) + ' Lat = ' +str(latitude) + ' Alt = ' + str(altitude))
		print('---------------------------------------------------------------------------------')
		rospy.loginfo(' Velocity X = '+ str(vel_x) + ' Y = ' +str(vel_y) + ' Z = ' + str(vel_z))
		
		#Sleep for 60/20 = 3s
		try:
			rate.sleep()
		except rospy.ROSInterruptException:
			pass		
