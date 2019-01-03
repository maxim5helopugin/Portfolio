#!/usr/bin/env python
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from quad.msg import Target_coordinates

from vision import Detector, profiles
from timer import Timer

# References: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# http://www.pirobot.org/blog/0016/

class VisionNode:
    def __init__(self):
        rospy.init_node('VisionNode')
        self.t = Timer('Detection, full')
        self.frame_num = 0      # frame

        self.bridge = CvBridge()
        self.sub_video = rospy.Subscriber('/videofile/image_raw', Image, self.detect)
        self.pub_detection = rospy.Publisher('detection', Target_coordinates, queue_size=1)
        self.pub_fpv = rospy.Publisher('fpv', Image, queue_size=1)
        self.detector = Detector(750, t=40)

        # Load detection profiles
        self.detector.load_profile(profiles.sim_drone)
        #self.detector.load_profile(profiles.idea_orange)
        #self.detector.load_profile(profiles.quadbox_white)

        self.iw = 'QUAD Vision'
        rospy.spin()
        cv2.namedWindow(self.iw, 0)
        #rospy.spin()

    def detect(self, input_image):
        """Run detection algorithm on image"""
        self.t.start()
        frame = self.convert_image(input_image)
        frame = cv2.pyrDown(frame)

        img, confidence, x, y = self.detector.detect(frame)
        print('Detection:', confidence, x, y)
        det = Target_coordinates()
        det.confidence = confidence
        det.x = x
        det.y = y
        self.pub_detection.publish(det)
        self.pub_fpv.publish(self.bridge.cv2_to_imgmsg(img))
        cv2.imwrite('frames/frame%d.jpg' % self.frame_num, img)
        self.frame_num += 1
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
