#!/usr/bin/env python3
#
# Detection viewer for ME134. Overlays detector output with camera images.
#

import numpy as np
import rospy
import time
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detector_demo.msg import ManyDetections


class Display:
    def __init__(self):
        rospy.init_node('display')

        # Prepare OpenCV bridge used to translate ROS Image message to OpenCV
        # image array (and back).
        self.bridge = CvBridge()

        self.img = np.zeros((640, 480, 3))

        # Prepare the detection image publisher.
        self.images_pub = rospy.Publisher(
            'detection_images', Image, queue_size=10)

        self.servo = rospy.Rate(100)

    def image_callback_method(self, data):
        # Set the saved image to the most recent recieved image.
        self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detection_callback_method(self, data):
        # Draw rectangles on the most recent image corresponding to the
        # detections returned by the detector in the detector message.
        img_out = self.img[:, :, :]
        for detection in data.detections:
            p1 = (int(detection.x - detection.size_x / 2),
                  int(detection.y - detection.size_y / 2))
            p2 = (int(detection.x + detection.size_x / 2),
                  int(detection.y + detection.size_y / 2))
            color = (0, 0, 255)
            thickness = 3
            img_out = cv2.rectangle(img_out, p1, p2, color, thickness)

        # Publish the resulting image (to be viewed by rqt_image_view)
        self.images_pub.publish(self.bridge.cv2_to_imgmsg(img_out, 'bgr8'))

    def start(self):
        # Start the image subscriber (listen for images from camera node).
        rospy.Subscriber('/usb_cam/image_raw', Image,
                         self.image_callback_method)
        # Start the detection subscriber (listen for detections from
        # detector node).
        rospy.Subscriber('/detections', ManyDetections,
                         self.detection_callback_method)

        while not rospy.is_shutdown():
            self.servo.sleep()


if __name__ == '__main__':
    d = Display()
    d.start()
