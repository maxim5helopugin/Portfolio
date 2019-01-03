#!/usr/bin/env python
# -*- coding: utf-8 -*-
### Just a translator. Most functions are taken from the dronkit library
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import sys
import rospy
from quad.msg import Vector_of_movement
from quad.msg import Sonars
from sensor_msgs.msg import Range
import argparse  

##Return a location object for a point in space
def get_location_metres(original_location, dNorth, dEast):
	return LocationGlobal(original_location.lat + (dNorth/6378137.0 * 180/math.pi), 
	original_location.lon + (dEast/(6378137.0*math.cos(math.pi*original_location.lat/180)) * 180/math.pi),
	original_location.alt)
###############################################

##Sends the x,y,z velocities to the copter
def send_global_velocity(velocity_x, velocity_y, velocity_z,refreshrate,heading=0):
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 				# type of the message : yaw control
        0, heading,    												# param 1, yaw in degrees
        0,          												# param 2, yaw speed deg/s
        1,         													# param 3, direction -1 ccw, 1 cw
        False, 														# param 4, relative offset 1, absolute angle 0
        0, 0, 0)    												# param 5 ~ 7 not used
    vehicle.send_mavlink(msg)										# send msg
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # type of the message : speed control
        0b0000111111000111, 0, 0, 0, 								# mask/ not used params
        velocity_x, velocity_y, velocity_z, 						# x,y,z velocities
        0, 0, 0, 0, 0)    											# not used params
    vehicle.send_mavlink(msg)										# send msg
											# sleep for the refrate
###############################################

##Ultrasonic subscriber, if one of the sensors is vithin the treshhold, panic
def ultrasonic(data):
	global sonar_alert
##	if data.sonar1<2 or data.sonar2<2 or data.sonar3<2 or data.sonar4 <2:
	if data.range<2:
		sonar_alert = 1
	else:
		sonar_alert = 0
###############################################

##Listen to the commands, send them to the copter
def callback(data):

	angle = (vehicle.heading)*3.14159265/180.0
	x = math.cos(angle)												# x and y are going to be used for the next
	y = math.sin(angle)												# heading calculations
	msg = Vector_of_movement()
	msg.x = x
	msg.y = y
	msg.z = 0

	global ignore, timeout

	if data.flag == 0:												# modes: 0 - search, 1 - pursuit, 2 - ignore
		print("Search mode")
		print("\t moving to ", x , ' ', y)
		vehicle.mode = VehicleMode('AUTO')
		if vehicle.commands.next >= 3:
			 vehicle.commands.next = 1
	elif data.flag == 1 and ignore == 0:
		print("Pursue mode")
		if vehicle.location.global_relative_frame.alt <= 1 or sonar_alert ==1:
			vehicle.mode = VehicleMode('AUTO')
			ignore = 1												# if in panic, ignore next n commands
			timeout = 50											# n = 50
		else:
			vehicle.mode = VehicleMode('GUIDED')
			thetha = math.degrees(math.atan(data.y/data.x))
			if data.y<0 and data.x > 0:								# calculate thetha
				thetha +=360;
			elif data.y>0 and data.x>0:
				thetha = thetha
			else:
				thetha +=180
			send_global_velocity(data.x*data.speed,data.y*data.speed,-data.z*data.speed,ref, thetha)
			msg.x = data.x 											# modify x and y and send to the guidance
			msg.y = data.y
			msg.z = data.z
			print("\t moving to ", msg.x , ' ', msg.y)
	else:
		print("Forced search mode") 
		if vehicle.commands.next >= 3:
			 vehicle.commands.next = 1
		timeout = timeout - 1										# decrease the counter to be ignored
		print('\t',timeout, " iterations left")												# print timeout
		if timeout <= 0:
			ignore = 0
	pub.publish(msg)
###############################################

##Listener initialization
def listener():
    flag = 0
    rospy.init_node('listener', anonymous=True)						# initialize 2 listeners, sonar and guidance
    rospy.Subscriber("sonar", Range, ultrasonic) # ROS topic , msg_type, Function
##    rospy.Subscriber("sonars", Sonars, ultrasonic) # ROS topic , msg_type, Function
    rospy.Subscriber("vector_of_movement", Vector_of_movement, callback)
    rospy.spin()
###############################################

##Arm and takeoff
def arm_and_takeoff(aTargetAltitude):
   	print ("Basic pre-arm checks")
   	while not vehicle.is_armable:									# wait until is armable
		print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
	print ("Arming motors")
	vehicle.mode = VehicleMode("GUIDED")							# guided takeoff
	vehicle.armed = True

	while not vehicle.armed:      
		print (" Waiting for arming...")
		time.sleep(1)

	print ("Taking off!")
	vehicle.simple_takeoff(aTargetAltitude) 						# Take off to target altitude
	while True:
		print (" Altitude: ", vehicle.location.global_relative_frame.alt)      
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
			print ("Reached target altitude")
			break
		time.sleep(1)
###############################################

##Waypoint map
def add_mission(aLocation, aSize, alt):
	cmds = vehicle.commands
	pi = 3.14159265/180.0
	angle = (vehicle.heading)*pi
	print (" Clear any existing commands")
	cmds.clear() 
    
	print (" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the two MAV_CMD_NAV_WAYPOINT locations and add the commands
	x = aSize*math.cos(angle) + 0*(-1*(math.sin(angle)))
	y = aSize*math.sin(angle) + 0*math.cos(angle)
	point1 = get_location_metres(aLocation, 0, 0)
	point2 = get_location_metres(aLocation, x, y)
	point3 = get_location_metres(aLocation, 0, 0)
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, alt))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, alt))
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, alt))  

	print (" Upload new commands to vehicle")
	cmds.upload()

###############################################

global sonar_alert
global ignore
global timeout 

ignore = 0
sonar_alert = 0
rate = int(sys.argv[1])
ref = 1.0/rate 														# set the refreshrate
target_altitude = 2 												# set the target altitude (search)
connection_string = "udp:127.0.0.1:14551"							# by default connect to itself
sitl = None
print('Connecting to vehicle on: %s' % connection_string)
if int(sys.argv[2]) == 1:											# if argument is 1 then connect to the pixhawk
	vehicle = connect("/dev/ttyUSB0", baud=1500000, wait_ready=True)
else:
	vehicle = connect(connection_string, wait_ready=True)

arm_and_takeoff(target_altitude)									# takeoff
pub = rospy.Publisher('previous_vector', Vector_of_movement, queue_size = 5)

add_mission(vehicle.location.global_frame,10, target_altitude)
vehicle.commands.next=0
vehicle.mode = VehicleMode("AUTO")

print("Set groundspeed to 1m/s.")
vehicle.groundspeed=1
listener()

vehicle.mode = VehicleMode("RTL")
print("Close vehicle object")
vehicle.close()
if sitl is not None:
    sitl.stop()
print("Completed")
