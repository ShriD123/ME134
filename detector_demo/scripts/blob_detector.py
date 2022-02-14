#!/usr/bin/env python3
#
# Example blob detector for ME134
#

import numpy as np
import rospy
import time
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detector_demo.msg import SingleDetection, ManyDetections


class Detector:

    # h_lims: (min, max) hue of object to be detected
    # s_lims (optional): (min, max) saturation of object to be detected
    # v_lims (optional): (min, max) value of object to be detected
    # detection_interval (optional): how often to run the detector
    def __init__(self,
                 h_lims, s_lims=None, v_lims=None, detection_interval=None):
        rospy.init_node('template_detector')

        # Prepare the detection publisher and detection message.
        # See Msg/SingleDetection.msg and Msg/ManyDetections.msg for details.
        self.detections_pub = rospy.Publisher(
            'detections', ManyDetections, queue_size=10)

        # TODO
        self.images_pub = rospy.Publisher(
            'detection_images_thresh', Image, queue_size=10)

        self.detections_msg = ManyDetections()

        # Prepare OpenCV bridge used to translate ROS Image message to OpenCV
        # image array.
        self.bridge = CvBridge()

        # Set detector parameters.
        upper_lims = []
        lower_lims = []
        for lim in [h_lims, s_lims, v_lims]:
            if lim is not None:
                lower_lims.append(lim[0])
                upper_lims.append(lim[1])
            else:
                lower_lims.append(0)
                upper_lims.append(255)
        self.upper_lims = tuple(upper_lims)
        self.lower_lims = tuple(lower_lims)

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

        # Convert BGR image to HSV.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image by the max/min Hue, Saturation, Value given.
        # Be careful with Hue as it wraps around!
        if (self.upper_lims[0] < self.lower_lims[0]):
            wrap_upper_lims = (180, self.upper_lims[1], self.upper_lims[2])
            wrap_lower_lims = (0, self.lower_lims[1], self.lower_lims[2])
            thresh = cv2.inRange(hsv, self.lower_lims, wrap_upper_lims) + \
                cv2.inRange(hsv, wrap_lower_lims, self.upper_lims)
        else:
            thresh = cv2.inRange(hsv, self.lower_lims, self.upper_lims)

        # Remove noise from the image via erosion/dilation.
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.erode(thresh, kernel, iterations=5)
        thresh = cv2.dilate(thresh, kernel, iterations=5)
        
        # Publish the image viewed by the camera
        self.images_pub.publish(self.bridge.cv2_to_imgmsg(thresh, 'mono8'))

        # Find contours within the thresholded image
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Select only the circular contours
        # Note, this isn't a great way of doing this, and is taken from here:
        # https://www.authentise.com/post/detecting-circular-shapes-using-contours
        # Hough circles are likely better and easier way of detecting circles
        # specifically (the contour method can easily be adapted to rectangles or
        # other shapes, however). For an example of Hough circles, see:
        # https://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
        circular_contours = []
        for contour in contours:
            approx = cv2.approxPolyDP(
                contour, 0.01*cv2.arcLength(contour, True), True)
            area = cv2.contourArea(contour)
            if ((len(approx) > 8) & (area > 30)):
                circular_contours.append(contour)

        # Construct detection message.
        self.detections_msg.detections = []
        for contour in circular_contours:
            x, y, w, h = cv2.boundingRect(contour)
            detection_msg = SingleDetection()

            detection_msg.x = x + w / 2
            detection_msg.y = y + h / 2
            detection_msg.size_x = w
            detection_msg.size_y = h

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
    d = Detector(h_lims=(20, 40), s_lims=(80, 150), v_lims=(0, 255))
    d.start()
