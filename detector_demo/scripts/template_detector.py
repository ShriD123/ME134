#!/usr/bin/env python3
#
# Example template detector for ME134
# See https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
#

import numpy as np
import rospy
import time
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detector_demo.msg import SingleDetection, ManyDetections


class Detector:

    # template_path: location of template image to match
    # thesh: threshold above which a match will be counted
    # detection_interval (optional): how often to run the detector
    def __init__(self,
                 template_path='/home/me134/me134ws/src/detector_demo/templates/template.png',
                 thresh=0.5, detection_interval=None):
        rospy.init_node('template_detector')

        # Import the template, make it grayscale, and record the width
        # and height.
        self.template = cv2.imread(template_path)
        self.template_gray = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
        self.template_width, self.template_height = self.template_gray.shape

        # Prepare the detection publisher and detection message.
        # See Msg/SingleDetection.msg and Msg/ManyDetections.msg for details.
        self.detections_pub = rospy.Publisher(
            'detections', ManyDetections, queue_size=10)
        self.detections_msg = ManyDetections()

        # Prepare OpenCV bridge used to translate ROS Image message to OpenCV
        # image array.
        self.bridge = CvBridge()

        # Set detector parameters.
        self.thresh = thresh
        self.detection_interval = detection_interval
        self.last_detection = rospy.Time.now()

        self.servo = rospy.Rate(100)

    def image_callback_method(self, data):
        # Check if the detector should be run (has long enough passed since
        # the last detection?).
        if self.detection_interval is not None \
                and (rospy.Time.now() - self.last_detection).to_sec() < self.detection_interval:
            return

        self.last_detection = rospy.Time.now()

        # Convert ROS Image message to OpenCV image array.
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Convert image to grayscale.
        img_gray = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_BGR2GRAY)

        # Match template. Result is an array of size
        # (image_width - template_width, image_height - template_height),
        # where each pixel denotes the degree of match between the template and
        # image at that location.
        res = cv2.matchTemplate(
            img_gray, self.template_gray, cv2.TM_CCOEFF_NORMED)

        # Get all locations where the degree to which the template matches
        # exceeds the threshold.
        detections = np.where(res >= self.thresh)
        xs = detections[0] + (self.template_width / 2)
        ys = detections[1] + (self.template_height / 2)

        # Construct detection message.
        self.detections_msg.detections = []
        for i in range(len(xs)):
            detection_msg = SingleDetection()

            detection_msg.x = xs[i]
            detection_msg.y = ys[i]
            detection_msg.size_x = self.template_width
            detection_msg.size_y = self.template_height

            self.detections_msg.detections.append(detection_msg)

        # Send detection message.
        self.detections_pub.publish(self.detections_msg)

    def start(self):
        # Start the image subscriber (listen for images from camera node).
        rospy.Subscriber('/usb_cam/image_raw', Image,
                         self.image_callback_method)

        while not rospy.is_shutdown():
            self.servo.sleep()


if __name__ == '__main__':
    d = Detector()
    d.start()
