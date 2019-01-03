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
import argparse  


##Return a location object for a point in space
def get_location_metres(original_location, dNorth, dEast):

	earth_radius=6378137.0
	dLat = dNorth/earth_radius
	dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
	newlat = original_location.lat + (dLat * 180/math.pi)
	newlon = original_location.lon + (dLon * 180/math.pi)
	return LocationGlobal(newlat, newlon,original_location.alt)
####################################


##Adjust yaw
def condition_yaw(heading=0, relative=True, ccw=1):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        ccw,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
################################

##Arm and takeoff
def arm_and_takeoff(aTargetAltitude):
   	print ("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
   	while not vehicle.is_armable:
		print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
	print ("Arming motors")
    # Copter should arm in GUIDED mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:      
		print (" Waiting for arming...")
		time.sleep(1)

	print ("Taking off!")
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
	while True:
		print (" Altitude: ", vehicle.location.global_relative_frame.alt)      
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
			print ("Reached target altitude")
			break
		time.sleep(1)
###################################

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

###################################

target_altitude = 1


##init the connection string (loopback for now, pixhawk later)
connection_string = "udp:127.0.0.1:14551"
sitl = None

print('Connecting to vehicle on: %s' % connection_string)
if int(sys.argv[1]) == 1:
	vehicle = connect("/dev/ttyUSB0", baud=1500000, wait_ready=True)
else:
	vehicle = connect(connection_string, wait_ready=True)
arm_and_takeoff(target_altitude)

add_mission(vehicle.location.global_frame,10, target_altitude)
vehicle.commands.next=0
vehicle.mode = VehicleMode("AUTO")

while True:
	if vehicle.commands.next == 3:
		vehicle.commands.next = 1

print("Set groundspeed to 1m/s.")
vehicle.groundspeed=0.5


vehicle.mode = VehicleMode("RTL")
print("Close vehicle object")
vehicle.close()
if sitl is not None:
    sitl.stop()
print("Completed")
