''' 
	The program will execute after master-node is running
	This will control UAV2 to traverse waypoints entered after running
'''

import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3Stamped
from sensor_msgs.msg import NavSatFix
#from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State 
from mavros_msgs.srv import *


def initUAV(uav):
	#getting current states
	uav.setStateSub(uav.currState) 

	#Assign arm and mode client
	uav.initArmingClient() 
	uav.initModeClient()	

	#getting local vel and pos params continuously
	uav.setLocalVelSub(uav.localVelocity)
	uav.setLocalPosSub(uav.localPosition)

	#Assign local pos, vel & accel publishers
	uav.initLocalPosPub() 
	uav.initLocalVelPub()
	uav.initLocalAccelPub()

	#setting sleeping frequency
	uav.setSleepRate( uav.freq)

	#getting global pos params 
	uav.setGPSPosSub(uav.globalPosition)

class pathPoints(object):
	'''Collecting waypoints and storing them in a list'''
	def __init__(self):
		self.entries = 0
		self.path_entries = []

	def getPathEntries(self):
		num = input("Enter number of path points.....")
		self.entries = int(num)

		for i in range(0, num):
			print("Enter x y z coordinates...")
			x, y, z = [float(x) for x in raw_input().split()]
			hold_time = input("Enter the holding time....")
			self.path_entries.append([False, x, y, z, int(hold_time)]) 


class uav(object):
	
	def __init__(self, name):
		''' All intrinsic parameters of UAV'''
		self.name = name

		#state variables
		self.state_arm = None
		self.state_connected = None
		self.state_mode = None

		#initialization 
		self.arming_client = None
		self.mode_client = None

		#getting local linear velocity values from publisher
		self.l_vel_x = None
		self.l_vel_y = None
		self.l_vel_z = None
		
		#getting local angular velocity values from publisher
		self.ang_vel_x = None
		self.ang_vel_y = None
		self.ang_vel_z = None

		self.vel_vec2 = None
		self.vel_vec3 = None
		self.accel_vec = None

		#getting local position values from publisher
		self.l_x = None
		self.l_y = None
		self.l_z = None

		#publisher variables
		self.local_pos_pub = None
		self.local_vel_pub = None
		self.local_accel_pub = None

		#Object for publishing position, velocity and acceleration
		self.pose = PoseStamped()
		self.v = Twist()
		self.accel = Vector3Stamped()

		#direction parameters for waypoint
		self.r = 0
		self.theta = 0
		self.phi = 0
		
		''' Pre-defined values'''
		self.flightH = 2 # initial hovering height 
		self.v_const = 0.4 # UAV tranverse with this constant velocity 
		self.freq = 20 # Frequency of program running 
		self.acc = 0.4 # Accuracy for a waypoint

		self.R = 0.3 #0.25 actual Iris radius

		# Time variables
		self.rate = None
		self.time_stamp = None

		# Variables for GPS coordinates
		self.latitude = None
		self.longitude = None
		self.altitude = None

		# Approximate cartesian coordinates derived from GPS
		self.gps_x = None
		self.gps_y = None
		self.gps_z = None

		# GPS coordinates of Home location
		self.home_x = 0
		self.home_y = 0
		self.home_z = 0

		# Variables for relative parameters
		self.los_dist = 0
		self.los_theta = 0
		self.los_phi = 0

		self.Vr = 0
		self.Vtheta = 0 
		self.Vphi = 0

		''' Assumed values '''
		self.k = 7
		self.delta = 1.570 #azimuth
 		self.gamma = 0.0	#elevation 
		self.approx = 2

#
	def setSleepRate(self, freq):
		''' binding object and defining frequency of sleep'''
		self.rate = rospy.Rate(freq)

#
	def setStateSub(self, method ):
		''' Subscribing to connection/arming/mode info '''
		rospy.Subscriber('/'+ self.name +'/mavros/state', State, method)

	def currState(self, msg):
		''' Continuously updating following params '''
		self.state_connected = msg.connected
		self.state_arm = armed = msg.armed
		self.state_mode = msg.mode

#
	def initArmingClient(self):
		''' assigning arming client to class variable '''
		self.arming_client = rospy.ServiceProxy('/'+self.name+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

	def initModeClient(self):
		''' assigning client of Arm services to class variable'''
		self.mode_client = rospy.ServiceProxy('/'+self.name+'/mavros/set_mode', mavros_msgs.srv.SetMode)

#
	def setLocalVelSub(self, method ):
		''' Subscribing to local velocity publisher'''
		rospy.Subscriber("/"+self.name+"/mavros/local_position/velocity", TwistStamped, method)

	def localVelocity(self, val):
		'''Local velocity fused by FCU '''
		vel_x = val.twist.linear.x
		vel_y = val.twist.linear.y
		vel_z = val.twist.linear.z

		ang_vel_x = val.twist.angular.x
		ang_vel_y = val.twist.angular.y
		ang_vel_z = val.twist.angular.z

		vel_vec2 = (vel_x**2 + vel_y**2)**0.5
		vel_vec3 = (vel_x**2 + vel_y**2 + vel_z**2)**0.5

		self.l_vel_x = round(vel_x, self.approx)
		self.l_vel_y = round(vel_y, self.approx)
		self.l_vel_z = round(vel_z, self.approx)

		self.ang_vel_x = round(ang_vel_x, self.approx)
		self.ang_vel_y = round(ang_vel_y, self.approx)
		self.ang_vel_z = round(ang_vel_z, self.approx)

		self.vel_vec2 = round(vel_vec2, self.approx)
		self.vel_vec3 = round(vel_vec3, self.approx)

#
	def setLocalPosSub(self, method ):
		''' Subscribing to local position publisher'''
		rospy.Subscriber("/"+self.name+"/mavros/local_position/pose", PoseStamped, method)

	def localPosition(self, val):
		''' getting updated local velocity '''
		local_x = val.pose.position.x
		local_y = val.pose.position.y
		local_z = val.pose.position.z
		
		self.l_x = round(local_x, self.approx)
		self.l_y = round(local_y, self.approx)
		#z will value be calculated using GPS and home altitude
		

#
	def initLocalPosPub(self, queue = 16):
		''' Assigning publisher to publish local position'''
		self.local_pos_pub = rospy.Publisher('/'+self.name+'/mavros/setpoint_position/local', PoseStamped, queue_size = queue)

	def initLocalVelPub(self, queue = 16):
		''' Assigning publisher to publish local velocity'''
		self.local_vel_pub = rospy.Publisher('/'+self.name+'/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = queue)

	def initLocalAccelPub(self, queue = 16):
		''' Assigning publisher to publish local acceleration'''
		self.local_accel_pub = rospy.Publisher('/'+self.name+'/mavros/setpoint_accel/accel', Vector3Stamped, queue_size = queue)

#
	def getStateConnected(self):
		''' return current status of connection between '''
		return self.state_connected

	def getStateMode(self):
		''' return current state'''
		return self.state_mode

	def getStateArm(self):
		''' return current status of arm '''
		return self.state_arm

#
	def setArm(self, cmd=True):
		'''Arm or disarm and return true if cmd executed '''
		return self.arming_client(cmd).success

	def setMode(self, mode="OFFBOARD"):
		'''return true if mode is set'''	
		return self.mode_client(custom_mode = mode)

#
	def verifyConnection(self):
		''' Wait till connection between MAVlink and autopilot is done'''
		try:
			while((not rospy.is_shutdown()) and (not self.getStateConnected())): 
				print('---Waiting for connection---')
				self.rate.sleep()        
		except rospy.ROSInterruptException:
			pass

		rospy.loginfo('Connection established')

#
	def setLocalPos(self, x=0, y=0, z=0):
		''' Setting object position'''
		self.pose.pose.position.x = x
		self.pose.pose.position.y = y
		self.pose.pose.position.z = z

	def publishPos(self):
		''' Publish local position on topic '''
		self.local_pos_pub.publish(self.pose)

#
	def calVelParam(self, other_x, other_y, other_z):
		''' calculating dist, azimuth and elevation angle'''

		r = ((other_x - self.l_x)**2 + (other_y - self.l_y)**2 + (other_z - self.l_z)**2)**0.5

		theta = math.atan2(other_y - self.l_y, other_x - self.l_x)
		
		try:
			phi = math.asin((other_z - self.l_z)/ r)
		except ZeroDivisionError:
			phi = 0

		self.r = round(r, self.approx)
		self.theta = round(theta, self.approx)
		self.phi = round(phi, self.approx)

	def getSepVel(self):
		''' calculating velocity components along individual axises'''
		vx = self.v_const * math.cos(self.phi) * math.cos(self.theta)
		vy = self.v_const * math.cos(self.phi) * math.sin(self.theta) 
		vz = self.v_const * math.sin(self.phi)
		return round(vx, self.approx), round(vy, self.approx), round(vz, self.approx)

#
	def setLocalVel(self, vx=0, vy=0, vz=0, ax = 0, ay = 0, az = 0):
		''' setting object velocity'''
		self.v.linear.x = vx
		self.v.linear.y = vy
		self.v.linear.z = vz

		self.v.angular.x = ax
		self.v.angular.y = ay
		self.v.angular.z = az

	def publishVel(self):
		''' Publish local velocity on topic'''
		self.local_vel_pub.publish(self.v)

#
	def setGPSPosSub(self, method ):
		''' Subscribing to global gps locations'''
		rospy.Subscriber("/"+ self.name +"/mavros/global_position/global", NavSatFix, method)

	def globalPosition(self, reading):
		'''Read GPS position fix reported by the device and calculate cartesian coordinates'''
		self.latitude = reading.latitude
		self.longitude = reading.longitude
		self.altitude = reading.altitude
		y = 6371*1000*math.cos(0.01745329252*reading.latitude)*math.cos(0.01745329252*reading.longitude)
		x = 6371*1000*math.cos(0.01745329252*reading.latitude)*math.sin(0.01745329252*reading.longitude)
		z = reading.altitude
		self.gps_x = round(x, self.approx)
		self.gps_y = round(-y, self.approx)
		self.gps_z = round(z, self.approx)

		#updating local coordinates with GPS and home coordinates
		self.l_z = round(self.gps_z - self.home_z, self.approx)

#
	def calCollParam(self, other):
		''' calculating LOS azimuth and elevation angle'''

		los_dist = ((other.gps_x - self.gps_x)**2 + (other.gps_y - self.gps_y)**2 + (other.gps_z - self.gps_z)**2)**0.5

		los_theta = math.atan2(other.gps_y - self.gps_y, other.gps_x - self.gps_x)
		
		try:
			los_phi = math.asin((other.gps_z - self.gps_z)/ los_dist)
		except ZeroDivisionError:
			los_phi = 0

		Vr = other.vel_vec3 * ( math.cos(other.phi) * math.cos(los_phi) * math.cos(other.theta - los_theta) + math.sin(other.phi) * math.sin(los_phi)) - self.vel_vec3 * (math.cos(self.phi) * math.cos(los_phi) * math.cos(self.theta - los_theta) + math.sin(self.phi) * math.sin(los_phi))	
	
		Vtheta = other.vel_vec3 * math.cos(other.phi) * math.sin( other.theta - self.los_theta) - self.vel_vec3 * math.cos(self.phi) * math.sin( self.theta - los_theta)

		Vphi = other.vel_vec3 * ( - math.cos(other.phi) * math.sin(los_phi) * math.cos(other.theta - los_theta) + math.sin(other.phi) * math.cos(los_phi) ) - self.vel_vec3 * (- math.cos(self.phi) * math.sin(los_phi) * math.cos(self.theta - los_theta) + math.sin(self.phi) * math.cos(los_phi))


		self.los_dist = round(los_dist, self.approx)
		self.los_theta = round(los_theta, self.approx)
		self.los_phi = round(los_phi, self.approx)

		self.Vr = round(Vr, self.approx)
		self.Vtheta = round(Vtheta, self.approx)
		self.Vphi = round(Vphi, self.approx)
		
#
	def collDetect(self):
		''' 
			The method evaluate the condition for y and Vr and give decision on collision 
			It returns True in case of collision and False otherwise			
		'''
		R = 2*self.R # Conpensating for point and sphere approximation 

		try:
			y = ((self.los_dist ** 2) * ((self.Vphi)**2 + (self.Vtheta)**2))/(self.Vtheta**2 + self.Vr**2 + self.Vphi**2) - R**2
		except ZeroDivisionError:
			y = self.R
				
		y = round(y, self.approx)

		rospy.loginfo('\n Component values.... Y(t) ='+ str(y) + '   Vr='+ str(self.Vr))

		if y <= 0.1 and self.Vr < 0.05:
			rospy.loginfo('\n!!!!!!! On collision course !!!!!!!!!!\n')
			return True
		else:
			rospy.loginfo('\n---------- Non-colliding ----------\n')
			return False

#
	def getSepAccel(self, z):
		'''			
			The method use relative parameters along LOS of UAVs and calculate the acceleration by guidance law
			It returns separate acceleration values along each axis			
		'''

		R = 2 * self.R # Conpensating for point and sphere approximation 

		try:
			accel_vec = (-0.5 * self.k * (self.Vtheta **2 + self.Vphi **2 + self.Vr**2)/self.los_dist**2) * (self.los_dist**2 * (self.Vtheta **2 + self.Vphi**2) - R**2 * (self.Vtheta**2 + self.Vphi**2 + self.Vr**2)) / (- self.Vtheta* self.Vr**2 * math.cos(self.gamma) * math.sin(self.delta - self.los_theta) + self.Vphi * self.Vr**2 *(math.cos(self.gamma) * math.sin(self.phi) * math.cos(self.delta - self.los_theta) - math.sin(self.gamma)* math.cos(self.los_phi)) + self.Vr * (self.Vtheta **2 + self.Vphi**2) * (math.cos(self.gamma) * math.cos(self.los_phi) * math.cos(self.delta - self.los_theta) + math.sin(self.gamma) * math.sin(self.los_phi) ) )
		except ZeroDivisionError:
			accel_vec = 0

		#approximating value
		self.accel_vec = round(accel_vec, self.approx)
	
		# Converting into cartesian coordinate with pre-defined angles		
		a_x = accel_vec * math.cos(self.gamma) * math.cos(self.delta)
		a_y = accel_vec * math.cos(self.gamma) * math.sin(self.delta)
		a_z = accel_vec * math.sin(self.gamma)  		
		

		return round(a_x, self.approx), round(a_y, self.approx), round(a_z, self.approx)

	def setLocalAccel(self, a_x=0, a_y=0, a_z=0):
		''' Setting object acceleration'''
		self.accel.vector.x = a_x
		self.accel.vector.y = a_y
		self.accel.vector.z = a_z

	def publishAccel(self):
		''' Publish local acceleration on topic'''
		self.local_accel_pub.publish(self.accel)


##
def flight(path, uav, other):

    ''' 
	The function do all initialization then traverse all waypoints with implementation of collision detection and guidance law for avoidance 
	'''
#	
	uav.verifyConnection()
#	

	uav.calVelParam(0 , 0, uav.flightH)
	vx, vy, vz = uav.getSepVel()
	uav.setLocalVel(vx,vy,vz)

	#Continuously publishing coordinates before flight
	try:
		for i in range(1,101):
			#uav.publishPos()
			uav.publishVel()			
			print('Publishing position coordinates')
			uav.rate.sleep()        
	except rospy.ROSInterruptException:
		pass

	uav.time_stamp = rospy.Time.now() #get stamp for next loop

	rec = True # for time stamping once

	uav.home_x, uav.home_y, uav.home_z  = uav.gps_x, uav.gps_y, uav.gps_z # taking record of altitude  

	# Following loop will arm, put UAV in offboard mode and let UAV reach flightH height before actual mission start		 
	while not rospy.is_shutdown():
		print('------------------In first main iteration--------------------------') #indicate process running

		if (uav.getStateMode() != "OFFBOARD") and (rospy.Time.now() - uav.time_stamp > rospy.Duration(2.0)): #Set mode to OFFBOARD if not
			print('-------Setting mode----------')
			if uav.setMode():
				rospy.loginfo('OFFBOARD enabled')
			uav.time_stamp = rospy.Time.now()

		elif (not uav.getStateArm()) and (rospy.Time.now() - uav.time_stamp > rospy.Duration(2.0)): #ARM the vehicle if not 
			print('-------Arming vehicle----------')
			if uav.setArm():
				rospy.loginfo('Vehicle armed')
			uav.time_stamp = rospy.Time.now()
		
		uav.calVelParam( 0, 0, uav.flightH)
		vx, vy, vz = uav.getSepVel()
		uav.setLocalVel( vx, vy, vz)
		uav.publishVel()		
		
		if uav.flightH - uav.acc <= uav.l_z and uav.l_z <= uav.flightH + uav.acc:
			''' UAV reached destination with appropriate accuracy'''

			if rec:		# recording time when reached flight height
				uav.time_stamp = rospy.Time.now()
				rec = False
		
			if rospy.Time.now() - uav.time_stamp >= rospy.Duration(8.0):
				''' UAV hold the position ''' 
				break #now uav will traverse through destination coordinates

		
		print('***** Current Local Position (%.3f %.3f %.3f) *****'% (uav.l_x, uav.l_y, uav.l_z))
		print('***** Current Velocities (%.3f  %.3f  %.3f) *****'% (uav.l_vel_x, uav.l_vel_y, uav.l_vel_z))
		print('***** Current Angular Velocities (%.3f  %.3f  %.3f) *****\n'% (uav.ang_vel_x, uav.ang_vel_y, uav.ang_vel_z))
		print('***** Current Global Position (%.3f %.3f %.3f) *****'% (uav.latitude, uav.longitude, uav.altitude))


		try:
			uav.rate.sleep()
		except rospy.ROSInterruptException:
			pass
	
	# mission points transversing starts
	for i in range(path.entries):
		x, y, z = path.path_entries[i][1], path.path_entries[i][2], path.path_entries[i][3] # assigning destination coordinates
		
		while not path.path_entries[i][0] :

			#calculating los and relative quantities between UAV and other
			uav.calCollParam(other)

			#calculating separate velocity to reach waypoint			
			uav.calVelParam( x, y, z)
			vx, vy, vz = uav.getSepVel()
			
			if uav.collDetect():
				ax, ay, az = uav.getSepAccel(z)
				print('\n***** Acceleration published (%.3f,  %.3f,  %.3f) *****\n'% (ax, ay, az))	
				vx, vy, vz = vx + ax, vy + ay, vz + az
		
			#setting velocity values and publishing
			uav.setLocalVel( vx, vy, vz)	
			uav.publishVel()

			# verifying conditions 
			x_bool = x - uav.acc <= uav.l_x and uav.l_x <= x + uav.acc
			y_bool = y - uav.acc <= uav.l_y and uav.l_y <= y + uav.acc
			z_bool = z - uav.acc <= uav.l_z and uav.l_z <= z + uav.acc

			#if destinations reached from all axises, start timer 
			if x_bool and y_bool and z_bool:
				
				time_completed = rospy.Time.now() - uav.time_stamp
				print('***** Time period covered = %s *****' % time_completed.secs)
			
				# hold duration covered exit loop and move to next coordinate
				if time_completed >= rospy.Duration(path.path_entries[i][4]):
					break
			else:
				''' updating time stamp, will stop updating when reached destination'''
				uav.time_stamp = rospy.Time.now()
	
			print('\n***** Current Position (%.3f %.3f %.3f) *****'% (uav.l_x, uav.l_y, uav.l_z))
			print('***** Destination Coordinates (%.3f %.3f %.3f) *****'% (x, y, z))
		#	print('***** Desitnation reached (%r  %r  %r)****' % (x_bool, y_bool, z_bool))

			print('***** Current Velocities (%.3f  %.3f  %.3f) *****'% (uav.l_vel_x, uav.l_vel_y, uav.l_vel_z))
		#	print('***** Current Angular Velocities (%.3f  %.3f  %.3f) *****'% (uav.ang_vel_x, uav.ang_vel_y, uav.ang_vel_z))

		#	print('***** Current Global Position (%.3f %.3f %.3f) *****'% (uav.latitude, uav.longitude, uav.altitude))

			print('\n***** LOS param (Dist = %.3f, Theta = %.3f, Phi = %.3f) *****'% (uav.los_dist, uav.los_theta, uav.los_phi))
			print('***** Relative param (Vr = %.3f, Vtheta = %.3f, Vphi = %.3f) *****\n'% (uav.Vr, uav.Vtheta, uav.Vphi))

			try:
				uav.rate.sleep()
			except rospy.ROSInterruptException:
				pass
		
#
if __name__ == '__main__':
	''' main loop'''
	
	rospy.init_node('offboard_node', anonymous = True)

	# creating a instance of UAV object
	uav1 = uav('uav1')
	initUAV(uav1)

	# creating another instance of UAV object
	uav2 = uav('uav2')
	initUAV(uav2)
	
	# creating a instance of path object
	path = pathPoints()
	path.getPathEntries()

	#flight 
	flight(path, uav2, uav1)	

	
