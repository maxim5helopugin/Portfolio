#!/usr/bin/env python
import sys, os, time
import gi
#import rospy   ### ROS ### 
gi.require_version('Gst', '1.0')
gi.require_version('Gtk', '3.0')
from gi.repository import Gst, GObject, Gtk

class GTK_Main:
	def __init__(self):
	
		# Path to the images
		imgPath = "images/droneImg (%1d).jpg"
		
		# Set up the gstreamer pipeline
		self.player = Gst.Pipeline.new("pipe-it-up")
		# Set up The source element 
		source = Gst.ElementFactory.make("multifilesrc", "QB")
		source.set_property("location", imgPath)
		source.set_property("index", 1)
		source.set_property("stop-index", 3000)
		
		#Create a capsfilter element for the source
		sourceCaps = Gst.ElementFactory.make("capsfilter", "sourceCaps")
		filter = Gst.caps_from_string("image/jpeg,width=640,height=480,framerate=30/1")
		sourceCaps.set_property("caps", filter)
		
		#Create additional elements for the pipeline
		jpegDec = Gst.ElementFactory.make("jpegdec", "dec")
		vidConvert = Gst.ElementFactory.make("videoconvert", "vidconvert")
		videoScale = Gst.ElementFactory.make("videoscale", "videoScale")
		queueEl = Gst.ElementFactory.make("queue", "queueEl")
		videoRate = Gst.ElementFactory.make("videorate", "videoRate")
		jpegEnc = Gst.ElementFactory.make("jpegenc", "jpegEnc")
		payload = Gst.ElementFactory.make("rtpjpegpay", "payload")
		
		#Set up the sink element 
		sink = Gst.ElementFactory.make("udpsink", "sink")
		sink.set_property("host", "192.168.0.7")
		sink.set_property("port", 5000)
		
		#Create the text overlay elements 
		altOverlay = Gst.ElementFactory.make("textoverlay","altOverlay")
		speedOverlay = Gst.ElementFactory.make("textoverlay", "speedOverlay")
		modeOverlay = Gst.ElementFactory.make("textoverlay", "modeOverlay")
		altOverlay.set_property("text", "Altitude: ")
		altOverlay.set_property("font-desc", "Sans, 16")
		altOverlay.set_property("valignment", 2)
		altOverlay.set_property("halignment", 0)
		speedOverlay.set_property("text", "Speed: ")
		speedOverlay.set_property("font-desc", "Sans, 16")
		speedOverlay.set_property("valignment", 2)
		speedOverlay.set_property("halignment", 1)
		modeOverlay.set_property("text", "Mode: ")
		modeOverlay.set_property("font-desc", "Sans, 16")
		modeOverlay.set_property("valignment", 2)
		modeOverlay.set_property("halignment", 2)

		#Test different encodings
		xEnc = Gst.ElementFactory.make("omxh264enc", "xEnc")
		hparse = Gst.ElementFactory.make("h264parse", "hparse")
		hpayload = Gst.ElementFactory.make("rtph264pay", "hpayload")

		#Add elements to the pipeline 
		self.player.add(source)
		self.player.add(sourceCaps)
		self.player.add(jpegDec)
		self.player.add(vidConvert)
		self.player.add(videoScale)
		self.player.add(altOverlay)
		self.player.add(speedOverlay)
		self.player.add(modeOverlay)
		self.player.add(queueEl)
		self.player.add(videoRate)
		self.player.add(xEnc)
		self.player.add(hparse)
		self.player.add(hpayload)
		#self.player.add(jpegEnc)
		#self.player.add(payload)
		self.player.add(sink)
		
		#Link the elements 
		source.link(sourceCaps)
		sourceCaps.link(jpegDec)
		jpegDec.link(vidConvert)
		vidConvert.link(videoScale)
		videoScale.link(altOverlay)
		altOverlay.link(speedOverlay)
		speedOverlay.link(modeOverlay)
		modeOverlay.link(queueEl)
		queueEl.link(videoRate)
		videoRate.link(xEnc)
		xEnc.link(hparse)
		hparse.link(hpayload)
		hpayload.link(sink)
		#jpegEnc.link(payload)
		#payload.link(sink)
		
		#Watch the bus
		bus = self.player.get_bus()
		bus.add_signal_watch()
		bus.enable_sync_message_emission()
		bus.connect("message", self.on_message)
		bus.connect("sync-message::element", self.on_sync_message)
		self.player.set_state(Gst.State.PLAYING)
		
		#Call function to update the pipeline
		fakeUpdatePipeline(self)
		# Subscribe to a ROS topic 
		#rospy.Subscriber("information", Information, callback)
	############### To work with ROS ######################
	#def updatePipeline(self): 
	#	rospy.spin()
	#	time.sleep(1)
	#	
	#def callback(self, data):
	#	self.player.get_by_name("altOverlay").set_property("text", "Altitude: " + str(data.alt) + " m")
	#	self.player.get_by_name("speedOverlay").set_property("text", "Speed: " + str(data.speed) + " m/s")
	#	self.player.get_by_name("modeOverlay").set_property("text", "Mode: " + str(data.mode))
	################ Uncomment when ROS is ready ##########
	
	####### Use to test pipeline update ##########
	def fakeUpdatePipeline(self):
		with open("data.txt", "r") as file: 
			altitude = file.readline()
		file.close()
		
		self.player.get_by_name("altOverlay").set_property("text", "Altitude: " + str(data.alt) + " m")
		time.sleep(1)
	
	def start_stop(self, w):
		pass
	def on_message(self, bus, message):
		pass 
	def on_sync_message(self, bus, message):
		pass

Gst.init(None)
GTK_Main()
GObject.threads_init()
Gtk.main()
