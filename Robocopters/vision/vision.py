#!/usr/bin/env python
"""Combines trained ORB feature detection and optical flow tracking """
from __future__ import print_function
from sys import version

from video import Video
from detection import Detector
from image_utils import *
from utils.timer import Timer
import profiles

ros_mode = False


def main(video=0, dowsample=True):
    print('QUAD Vision, Python:', version, '  OpenCV:', cv2.__version__)
    vid = Video(video, 'ORB Detection testing', delay=1, fast=False, skip=20, downsample=downsample)
    detector = Detector()

    if ros_mode:
        import rospy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge, CvBridgeError
        from quad.msg import Target_coordinates

        pub_detection = rospy.Publisher('detection', Target_coordinates, queue_size=5)
        pub_frame = rospy.Publisher('annotated_frame', msg.image, queue_size=15)
        rospy.init_node('detection', anonymous=True)
        rospy.init_node('annotated_frame', anonymous=True)

    # Detection profiles
    detector.load_profile(profiles.idea_orange)
    # detector.load_profile(profiles.quadbox_black)
    # detector.load_profile(profiles.quadbox_white)
    # detector.load_profile(profiles.idea_red)
    # detector.load_profile(profiles.sim_drone)

    vid.skip(20)
    t = Timer('Detection')

    while vid.next():
        frame = vid.img

        t.start()   # start timer
        img, confidence, x, y = detector.detect(frame)
        print('Detection:', confidence, x, y)
        t.end()
        vid.display(img)

        if ros_mode:
            detection = Target_coordinates()
            detection.confidence = confidence
            detection.x = x
            detection.y = y
            try:
                pub_detection.publish(detection)
            except CvBridgeError as e:
                print(e)
            # ros_img = CvBridge.cv2_to_imgmsg(img, encoding="passthrough")
            # pub_frame.publish(ros_img)

    vid.close()
    t.output()


if __name__ == "__main__":
    import sys
    video = 0
    downsample = True

    # video = "D:/VIDEO/wobble2.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/data/gazebo/PurplePrey1.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/data/gazebo/PurplePrey2.mp4"
    video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Courtyard multi.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/IDEA2/Orange/orangeA1.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/IDEA2/Orange/courtyard323.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/Prey/IDEA2/Orange/orange7_30am.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/Prey/IDEA2/Red/red_inside2.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/Prey/IDEA2/Orange/orange9am.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/Prey/IDEA2/Orange/orange10am2.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/Prey/IDEA2/Orange/old/field (1).mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/IDEA2/Orange/field (3).mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Quadbox/White/courtyard1.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Quadbox/White/courtyard2.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Quadbox/White/white9am.mp4"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Quadbox/Black/quadbox_close.MOV"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Quadbox/Black/Quadbox 2 1m21s.MOV"
    # video = "C:/Users/seanr/OneDrive/UAV/UAV/prey/Quadbox/Black/Quadbox 1 16s.MOV"
    if len(sys.argv) > 1:
        video = sys.argv[1]
    if len(sys.argv) > 2:
        downsample = (sys.argv[2] == 1)

    main(video, downsample)
