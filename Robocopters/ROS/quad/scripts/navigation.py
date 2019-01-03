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
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time
import math
import sys
import rospy
from quad.msg import Vector_of_movement
from quad.msg import Information
from quad.msg import Sonars
import argparse  

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2[0] - aLocation1.north
    dlong = aLocation2[1] - aLocation1.east
    return math.sqrt((dlat*dlat) + (dlong*dlong))

##Local search mode 
def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
###############################################

##Borders alarm
def borders():
	global borders_alert
	if vehicle.location.local_frame.down > -1 or vehicle.location.local_frame.down < -7:
		borders_alert = 1
	else:
		borders_alert = 0
###############################################

##Sends the x,y,z velocities to the copter
def send_ned_velocity(velocity_x, velocity_y, velocity_z,refreshrate,heading=0):
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 				# type of the message : yaw control
        0, heading,    												# param 1, yaw in degrees
        0,          												# param 2, yaw speed deg/s
        1,         													# param 3, direction -1 ccw, 1 cw
        False, 														# param 4, relative offset 1, absolute angle 0
        0, 0, 0)    												# param 5 ~ 7 not used
    vehicle.send_mavlink(msg)	
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)     
    vehicle.send_mavlink(msg)	    
###############################################

##Ultrasonic subscriber, if one of the sensors is within the treshhold, panic
def ultrasonic(data):
	global sonar_alert
	global sonar_error
	if data.sonar1<2 or data.sonar2<2 or data.sonar3<2 or data.sonar4 <2:	# one of the sonars beneath the treshold
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
	if(get_distance_metres(vehicle.location.local_frame, point1)<0.3):
		current_command = 2
	elif(get_distance_metres(vehicle.location.local_frame, point2)<0.3):
		current_command = 1

	angle = (vehicle.heading)*3.14159265/180.0
	x = math.cos(angle)												# x and y are going to be used for the next
	y = math.sin(angle)												# heading calculations
	msg = Vector_of_movement()
	stats = Information()
	msg.x = x
	msg.y = y
	msg.z = 0
	stats.alt = vehicle.location.local_frame.down*(-1)
	stats.speed = vehicle.groundspeed

	global ignore, timeout
	borders()
	if ignore == 1:
		stats.mode = 0
		print("Forced search mode") 
		if(current_command == 1):
			goto_position_target_local_ned(point1[0], point1[1], point1[2])
		else:
			goto_position_target_local_ned(point2[0], point2[1], point2[2])
		timeout = timeout - 1										# decrease the counter to be ignored
		print('\t',timeout, " iterations left, command :", current_command)
		if timeout <= 0:
			ignore = 0
	elif data.flag == 0:
		stats.mode = 1	
		print("Search mode")
		print("\t moving to command", current_command)										
		if(current_command == 1):
			goto_position_target_local_ned(point1[0], point1[1], point1[2])
		else:
			goto_position_target_local_ned(point2[0], point2[1], point2[2])
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
			print("z = ", data.z, "  z* speed = ", data.z*data.speed)
			send_ned_velocity(data.x*data.speed,data.y*data.speed,(-1)*data.z*data.speed,ref, thetha)
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
	rospy.Subscriber("sonars", Sonars, ultrasonic)
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
		if vehicle.location.local_frame.down<=(-1)*aTargetAltitude*0.95: #Trigger just below target alt.
			print ("Reached target altitude")
			break
		time.sleep(1)
###############################################

global sonar_alert, ignore, timeout, borders_alert, sonar_error, next_command
global point1, point2
global current_command

current_command = 1
sonar_error = 0														# there is no sonar error in the beginning
ignore = 1															# ignore is 1 to let the vehicle initialize properly
sonar_alert = 0														# sonar alert is 0, no obstacles around
borders_alert = 0													# we are within the bound in the beginning
timeout = 50														# amount of cycles ignored for the initialization
next_command = 1													# next command is the one we start with
target_altitude = 2 												# set the target altitude (search)

rate = int(sys.argv[1])
ref = 1.0/rate 														# set the refreshrate
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

point1 = [7, -3, -2]
point2 = [2, -1, -2]

listener()
