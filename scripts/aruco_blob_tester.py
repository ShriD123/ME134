#!/usr/bin/env python3
#
#   detectaruco.py
#
#   Run the ArUco detector on a single image.
#

# ROS Imports
import numpy as np
import rospy
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detector_demo.msg import SingleDetection, ManyDetections
from sensor_msgs.msg import CameraInfo
from ME134.msg import array
#import imutils          # Just for testing/rotating image.

class Detector:

    def __init__(self, detection_interval=None):
        rospy.init_node('aruco_detector')
        
        # Prepare the detection publisher and detection message.
        # See Msg/SingleDetection.msg and Msg/ManyDetections.msg for details.
        self.detections_pub = rospy.Publisher(
            'detections', ManyDetections, queue_size=10)

        # TODO
        self.images_pub = rospy.Publisher(
            'detection_images_thresh', Image, queue_size=10)

        self.ctr_pub = rospy.Publisher('aruco_center',array, queue_size=10)

        self.detections_msg = ManyDetections()

        # Prepare OpenCV bridge used to translate ROS Image message to OpenCV
        # image array.
        self.bridge = CvBridge()

        self.detection_interval = detection_interval
        self.last_detection = rospy.Time.now()

        self.servo = rospy.Rate(100)
    
    def aruco_detect(self, data):
    
        # Check if the detector should be run (has long enough passed since
        # the last detection?).
        if self.detection_interval is not None \
                and (rospy.Time.now() - self.last_detection).to_sec() < self.detection_interval:
            return

        self.last_detection = rospy.Time.now()

        # Convert ROS Image message to OpenCV image array.
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        # Convert BGR image to HSV.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
        # Grab the image.
        # image = cv2.imread("original.jpeg")

        # Show the original image.
        #cv2.imshow("Original Image", image)
        #cv2.waitKey(0)


        # Set up the ArUco detector.  Use a dictionary with the appropriate
        # tags.  DICT_6x6_1000 has 1000 options (way too many).  DICT_6x6_250
        # was used to create the markers in the test image (IDs up to 250).
        # DICT_6x6_50 is probably enough for our projects...
        dict_aruco   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        params = cv2.aruco.DetectorParameters_create()

        # Detect.
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, dict_aruco, parameters=params)
        print(corners)
        print(ids)
        
        if ids is None:
            return
        
        # Array that holds the center location of 4 aruco markers
        marker_id = np.zeros(5)
        center_holder = np.zeros(10)

        i = 0
        j = 0
        # Loop over each marker: the list of corners and ID for this marker.
        for (markercorners, markerid) in zip(corners, ids.flatten()):
            # The corners are top-left, top-right, bottom-right, bottom-left.
            # Each is a 2x1 numpy array of pixel coordiantes.  Their type is
            # floating-point though they contain integer values.  NOTE
            # top-left is *not* in the image, but the top-left corner of the
            # original marker, which may be rotated in the image.  But the
            # center remains the center of either diagonal.
            (topLeft, topRight, bottomRight, bottomLeft) = markercorners[0]
            center = (topLeft + bottomRight)/2

            # Create the pixel tuples, for the drawing functions below.
            tl  = tuple(topLeft.astype(int))
            tr  = tuple(topRight.astype(int))
            bl  = tuple(bottomLeft.astype(int))
            br  = tuple(bottomRight.astype(int))
            ctr = tuple(center.astype(int))             # Center point
            txt = (tl[0], tl[1] - 15)                   # Where to write ID#
            marker_id[j] = markerid
            center_holder[i] = ctr[0]
            center_holder[i+1] = ctr[1]

            # Pick the drawing colors.
            boxcolor    = (0, 255, 0)
            circlecolor = (0, 0, 255)
            textcolor   = (0, 255, 0)

            # Draw the bounding box
            cv2.line(image, tl, tr, boxcolor, 2)
            cv2.line(image, tr, br, boxcolor, 2)
            cv2.line(image, br, bl, boxcolor, 2)
            cv2.line(image, bl, tl, boxcolor, 2)

            # Draw the center circle
            cv2.circle(image, ctr, 4, circlecolor, -1)

            # Print the ArUco marker ID on the image
            cv2.putText(image, str(markerid), txt,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, textcolor, 2)

            print("ArUco marker ID:", markerid)
            i = i + 2
            j = j + 1

        # Show the output image
        # cv2.imshow("Processed Image", image)
        # Publish the resulting image (to be viewed by rqt_image_view)
        self.images_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

        self.ctr_pub.publish(marker_id, center_holder)
        # rospy.sleep(0.25)
        # cv2.waitKey(0)
    
    
    def start(self):
        # Start the image subscriber (listen for images from camera node).
        rospy.Subscriber('/usb_cam/image_raw', Image,
                         self.aruco_detect)

        while not rospy.is_shutdown():
            self.servo.sleep()


if __name__ == '__main__':
    d = Detector()
    d.start()
