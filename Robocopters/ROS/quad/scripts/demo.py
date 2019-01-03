#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import sys
import rospy
from quad.msg import Information

##Listener initialization
def listener():
	rospy.Subscriber("information", Information, callback)
	rospy.spin()

def callback(data):
	print("current flight info : ")
	print("\t altitude : ", data.alt)
	print("\t velocity : ", data.speed)
	print("\t mode :", data.mode)

rospy.init_node('print_info', anonymous=True)
listener()