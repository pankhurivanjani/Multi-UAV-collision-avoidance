#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
#from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import *
from mavros_msgs.msg import State 

#current_state = State()

#Initializing with opposite values
connected = False
armed = False
mode = "GUIDED" 

#method subscribed to get state data
def state_cb(msg):
	global connected
	global armed
	global mode
	connected = msg.connected
	armed = msg.armed
	mode = msg.mode
	
#main loop
if __name__ == '__main__':
	#initializing node 
	rospy.init_node('offboard_node', anonymous=True)
	#subscribing 
	rospy.Subscriber('/uav2/mavros/state', State, state_cb)
	#getting publisher of position, mode and arm
	local_pos_pub = rospy.Publisher('/uav2/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
	set_mode_client = rospy.ServiceProxy('/uav2/mavros/set_mode', mavros_msgs.srv.SetMode)
	arming_client = rospy.ServiceProxy('/uav2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
	#sleep rate 
	rate = rospy.Rate(20)

	#verifying connection
	try:
		while((not rospy.is_shutdown()) and (not connected)): #current_state removed
			print('---Waiting for connection---')
			rate.sleep()        
	except rospy.ROSInterruptException:
		pass	

	rospy.loginfo('Connection established')
	
	#assigning position coordinates
	pose = PoseStamped()
	pose.pose.position.x = 1 #Changed relatively with respect to V1
	pose.pose.position.y = 0 
	pose.pose.position.z = 2

	#Continuously publishing coordinates so that vehicle don't get disarm
	try:
		for i in range(1,101):
			local_pos_pub.publish(pose)
			print('Publishing position coordinates')
			rate.sleep()        
	except rospy.ROSInterruptException:
		pass
	
	#setting object values
	offb_set_mode = SetMode()
	offb_set_mode.custom_mode = 'OFFBOARD'	

	arm_cmd = CommandBool()
	arm_cmd.value = True	

	#time stamping
	last_request = rospy.Time.now()

	#Continue to give commands until armed and mode set
	while not rospy.is_shutdown():
		if (mode != "OFFBOARD") and (rospy.Time.now() - last_request > rospy.Duration(5.0)): #current_state removed
			print('Setting mode')
			if set_mode_client(custom_mode = "OFFBOARD"):
				rospy.loginfo('OFFBOARD enabled')
			last_request = rospy.Time.now()
		else:
			if (not armed) and (rospy.Time.now() - last_request > rospy.Duration(5.0)): #current_state removed
				print('Arming vehicle')
				if arming_client(True).success:
					rospy.loginfo('Vehicle armed')

		#publishing local coordinates
		local_pos_pub.publish(pose)

		try:
			rate.sleep()
		except rospy.ROSInterruptException:
			pass		
