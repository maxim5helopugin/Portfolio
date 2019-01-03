#!/usr/bin/env python
# -*- coding: utf-8 -*-
### Just a translator. Most functions are taken from the dronkit library
###
###
### needs:
### - ultrasonic prey ignore
### - border detection
### - track of unseen targets
############################################

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import sys
import rospy
from quad.msg import Vector_of_movement
from quad.msg import Information
from quad.msg import Sonars
from sensor_msgs.msg import Range
import argparse  


##Borders alarm
def borders():
	global borders_alert
	if vehicle.location.global_relative_frame.alt < 1 or vehicle.location.global_relative_frame.alt > 7:
		borders_alert = 1
	else:
		borders_alert = 0
###############################################

##New mission control commands
def goto_position_target_global_int(aLocation):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 			# frame
        0b0000111111111000, 										# type_mask (only speeds enabled)
        aLocation.lat*1e7, 											# lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, 											# lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, 												# alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
###############################################  

##Distance between the points function
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
####################################

##Return a location object for a point in space
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;
####################################

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
###############################################

##Ultrasonic subscriber, if one of the sensors is within the treshhold, panic
def ultrasonic(data):
	global sonar_alert
	global sonar_error
	##if data.sonar1<2 or data.sonar2<2 or data.sonar3<2 or data.sonar4 <2:	# one of the sonars beneath the treshold
        #if data.range<2:
        if data.range<2:
		sonar_error = sonar_error + 1										# increase the alarm_counter by 1
	else:
		sonar_error = 0

	if sonar_error > 3:														# if the readings are continuous 
		sonar_alert = 1														# alert 
	else:
		sonar_alert = 0
###############################################

##Listen to the commands, send them to the copter
def callback(data):
	global current_command

	if(get_distance_metres(vehicle.location.global_relative_frame, point1)<0.3):
		current_command = 2
	elif(get_distance_metres(vehicle.location.global_relative_frame, point2)<0.3):
		current_command = 1
	angle = (vehicle.heading)*3.14159265/180.0
	x = math.cos(angle)												# x and y are going to be used for the next
	y = math.sin(angle)												# heading calculations
	msg = Vector_of_movement()
	stats = Information()
	msg.x = x
	msg.y = y
	msg.z = 0
	stats.alt = vehicle.location.global_relative_frame.alt
	stats.speed = vehicle.groundspeed

	global ignore, timeout
	borders()
	if ignore == 1:
		stats.mode = 0
		print("Forced search mode") 
		if(current_command == 1):
			goto_position_target_global_int(point1)
		else:
			goto_position_target_global_int(point2)
		timeout = timeout - 1										# decrease the counter to be ignored
		print('\t',timeout, " iterations left, command :", current_command)
		if timeout <= 0:
			ignore = 0
	elif data.flag == 0:
		stats.mode = 1	
		print("Search mode")
		print("\t moving to command", current_command)										
		if(current_command == 1):
			goto_position_target_global_int(point1)
		else:
			goto_position_target_global_int(point2)

	else:
		stats.mode = 2 
		print("Pursue mode")
		if borders_alert==1 or sonar_alert ==1:
			ignore = 1												# if in panic, ignore next n commands
			timeout = 50											# n = 50
		else:
			if data.x == 0:
				data.x = data.x + 0.001
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
	pubInfo.publish(stats)		
	pub.publish(msg)
	sysrate.sleep()
###############################################

##Listener initialization
def listener():
##	rospy.Subscriber("sonars", Sonars, ultrasonic)
	rospy.Subscriber("sonar", Range, ultrasonic)
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

global sonar_alert, ignore, timeout, borders_alert, sonar_error, next_command
global point1, point2, current_command
current_command = 1

sonar_error = 0														# there is no sonar error in the beginning
ignore = 1															# ignore is 1 to let the vehicle initialize properly
sonar_alert = 0														# sonar alert is 0, no obstacles around
borders_alert = 0													# we are within the bound in the beginning (50)
timeout = 5														# amount of cycles ignored for the initialization
next_command = 1													# next command is the one we start with
target_altitude = 3 												# set the target altitude (search)

rate = int(sys.argv[1])
#ref = 1.0/rate
ref = 10 														# set the refreshrate
connection_string = "udp:127.0.0.1:14551"							# by default connect to itself
print('Connecting to vehicle on: %s' % connection_string)
if int(sys.argv[2]) == 1:											# if argument is 1 then connect to the pixhawk
	vehicle = connect("/dev/ttyS0", baud=57600, wait_ready=True)
else:
	vehicle = connect(connection_string, wait_ready=True)

arm_and_takeoff(target_altitude)									# arm and takeoff


pub = rospy.Publisher('previous_vector', Vector_of_movement, queue_size = 5)
pubInfo = rospy.Publisher('information', Information, queue_size = 2)
rospy.init_node('previous_vector', anonymous=True)
sysrate = rospy.Rate(rate)

print("Set groundspeed to 1m/s.")
vehicle.groundspeed=1

pi = 3.14159265/180.0
angle = (vehicle.heading-25)*pi
x = 2*math.cos(angle) + 0*(-1*(math.sin(angle)))
y = 2*math.sin(angle) + 0*math.cos(angle)
point1 = get_location_metres(vehicle.location.global_relative_frame, x, y)	# command 1 is 2m away
x = 10*math.cos(angle) + 0*(-1*(math.sin(angle)))
y = 10*math.sin(angle) + 0*math.cos(angle)
point2 = get_location_metres(vehicle.location.global_relative_frame, x, y)	# command 2 is 10m away

listener()
