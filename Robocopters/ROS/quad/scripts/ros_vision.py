#!/usr/bin/env python
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from quad.msg import Target_coordinates, Information

from vision import Detector, profiles
from timer import Timer


# References: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# http://www.pirobot.org/blog/0016/

class VisionNode:
    def __init__(self):
        rospy.init_node('VisionNode')
        self.t = Timer('Detection, full')
        self.frame_num = 0  # frame
        self.alt = 0
        self.speed = 0
        self.mode = 0
        self.time = 0
        self.pub_frequency = 3  # publish detected images every nth frame

        self.bridge = CvBridge()
        self.sub_video = rospy.Subscriber('/videofile/image_raw', Image, self.detect)
        self.sub_info = rospy.Subscriber('information', Information, self.update_info)
        self.pub_detection = rospy.Publisher('detection', Target_coordinates, queue_size=1)
        self.pub_fpv = rospy.Publisher('fpv', Image, queue_size=1)
        self.detector = Detector(800, t=39)

        # Load detection profiles
        self.detector.load_profile(profiles.sim_drone)
        #self.detector.load_profile(profiles.idea_orange)
        #self.detector.load_profile(profiles.quadbox_white)
        #self.detector.load_profile(profiles.quadbox_black)
        #self.detector.load_profile(profiles.idea_red)

        self.iw = 'QUAD Vision'
        rospy.spin()
        cv2.namedWindow(self.iw, 0)

    def detect(self, input_image):
        """Run detection algorithm on image"""
        if input_image is None:
            return
        self.t.start()
        frame = self.convert_image(input_image)
        frame = cv2.pyrDown(frame)

        img, confidence, x, y = self.detector.detect(frame)
        # print('Detection:', confidence, x, y)
        det = Target_coordinates()
        det.confidence = confidence
        det.x = y
        det.y = x
        self.pub_detection.publish(det)

        # FPV
        img = self.annotate_img(img)
        self.pub_fpv.publish(self.bridge.cv2_to_imgmsg(img))
        # if self.time % self.pub_frequency == 0:
        #     filename = '/home/pi/frames/frame%d.jpg' % self.frame_num
        #     print('Save image', filename)
            # try:
            #     cv2.imwrite(filename, cv2.pyrDown(img))
            # except:
            #     print("Couldn't save image")
            # self.frame_num += 1
        self.time += 1
        self.t.end()
        # Display
        cv2.imshow(self.iw, img)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            cv2.destroyAllWindows()
            sys.exit(27)

    def convert_image(self, ros_img):
        """Convert ROS image message to OpenCV format"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
            return cv_image
        except CvBridgeError as e:
            print(e)

    def update_info(self, info):
        """Update altitude and other flight information published by guidance system for display."""
        self.alt = info.alt
        self.speed = info.speed
        self.mode = info.mode

    def annotate_img(self, img):
        """Display flight information on image."""
        mode = 'Search'
        if self.mode == 0:
            mode = 'Forced Search'
        elif self.mode == 2:
            mode = 'Pursue'

        speed = int(self.speed)

        h = 15
        line_height = 18
        img = cv2.rectangle(img, (0, 0), (250, line_height * 3), (0, 0, 0), -1)

        img = cv2.putText(img, 'Mode: ' + str(mode), (15, h), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255))
        h += line_height
        img = cv2.putText(img, 'Altitude: ' + '%.3f' % self.alt + 'm', (15, h), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255))
        h += line_height
        img = cv2.putText(img, 'Speed: ' + str(speed) + ' %', (15, h), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255))
        return img


def main():
    vision = VisionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    cv2.destroyAllWindows()
    vision.t.output()


if __name__ == "__main__":
    main()
