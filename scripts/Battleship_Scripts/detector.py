#!/usr/bin/env python3

import rospy
import math
import kinematics as kin
import numpy as np

''' This code contains the functionality of the detector node as well as all the constituent params.'''


###############################################################################
#
#  Helper Functions
#
class Helper:
    #TODO
    # This should not be a class, but rather global functions. This is just
    # a placeholder as a reminder to add in helper functions (if necessary).

    #TODO: Add the error sensing functions here? Or in Detector Class?
    pass


###############################################################################
#
#  Detector Class
#
class Detector:

    #TODO: Adjust the following code for our purposes.
    #TODO: Add calibration 

    #
    # Initialize.
    #
    def __init__(self, detection_interval=None):
        rospy.init_node('aruco_detector')
        
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

        self.detection_interval = detection_interval
        self.last_detection = rospy.Time.now()

        self.servo = rospy.Rate(100)
    

    #
    # Detect the hacky sack (update this code for our purposes)
    #
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

        # Show the output image
        # cv2.imshow("Processed Image", image)
        # Publish the resulting image (to be viewed by rqt_image_view)
        self.images_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        # rospy.sleep(0.25)
        # cv2.waitKey(0)
    

    #
    # Calibrate the Detector
    #
    def calibrate(self):
        #TODO: Implement calibration at some interval (not same as servo rate necessarily)
        pass


    #
    # Determine the XYZ position (in task space) of some detected object
    #
    def find_xyz(self):
        #TODO: Implement using point cloud and ArUco later
        pass


    #
    # Start the Detector (not sure if we really need this function)
    #
    def start(self):
        # Start the image subscriber (listen for images from camera node).
        rospy.Subscriber('/usb_cam/image_raw', Image,
                         self.aruco_detect)

        while not rospy.is_shutdown():
            self.servo.sleep()
    

    ###############################################################################
    #
    #  Main Code
    #
    def main(self):
        #TODO: Implement this when encapsulating the functionality into the Detector object
        # for use in the Battleship class.
        pass



###############################################################################
#
#  Main Code for Detector Only
#
if __name__ == '__main__':
    d = Detector()
    d.start()