#!/usr/bin/env python3

from multiprocessing.sharedctypes import Value
import rospy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detector_demo.msg import SingleDetection, ManyDetections
from sensor_msgs.msg import CameraInfo
from ME134.msg import aruco_center

''' This code contains the functionality of the detector node as well as all the constituent params.'''


###############################################################################
#
#  Detector Class
#
class Detector:

    #
    # Initialize.
    #
    def __init__(self,h_lims, s_lims=None, v_lims=None,detection_interval=None):
        rospy.init_node('aruco_detector')

        # Grab an instance of the camera data with predetermined calibration.
        rospy.loginfo("Waiting for camera info...")
        msg = rospy.wait_for_message('/usb_cam/camera_info', CameraInfo)
        self.camD = np.array(msg.D).reshape(5)
        self.camK = np.array(msg.K).reshape((3,3))

        # Report (to see what is happening).
        rospy.loginfo("Received Distortion:    \n %s", self.camD)
        rospy.loginfo("Received Camera Matrix: \n %s", self.camK)
        rospy.loginfo("Image Center at (%7.2f, %7.2f)",
                      self.camK[0][2], self.camK[1][2])
        rospy.loginfo("FOV %6.2f deg horz, %6.2f def vert",
                      np.rad2deg(np.arctan(msg.width /2/self.camK[0][0])*2),
                      np.rad2deg(np.arctan(msg.height/2/self.camK[1][1])*2))

        # Arrays that hold the center location and ids of 4 aruco markers
        self.marker_id = np.zeros(4)
        self.center_values = np.zeros(8)
        
        # Prepare the detection publisher and detection message.
        # See Msg/SingleDetection.msg and Msg/ManyDetections.msg for details.
        self.detections_pub = rospy.Publisher('detections', ManyDetections, queue_size=10)

        self.images_pub = rospy.Publisher(
            'detection_images_thresh', Image, queue_size=10)

        self.detections_msg = ManyDetections()

        self.blobpub = rospy.Publisher('/blob_loc', aruco_center, queue_size=10)

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
        
        self.blob_sub = rospy.Subscriber('/usb_cam/image_raw', Image,self.find_xyz)
    

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
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        # Convert BGR image to HSV.
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
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
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.image, dict_aruco, parameters=params)
        #print(corners)
        #print(ids)
        
        if ids is None:
            raise ValueError('No Aruco Markers Detected!')
        
 

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
            self.marker_id[j] = markerid
            self.center_values[i] = ctr[0]
            self.center_values[i+1] = ctr[1]

            # Pick the drawing colors.
            boxcolor    = (0, 255, 0)
            circlecolor = (0, 0, 255)
            textcolor   = (0, 255, 0)

            # Draw the bounding box
            cv2.line(self.image, tl, tr, boxcolor, 2)
            cv2.line(self.image, tr, br, boxcolor, 2)
            cv2.line(self.image, br, bl, boxcolor, 2)
            cv2.line(self.image, bl, tl, boxcolor, 2)

            # Draw the center circle
            cv2.circle(self.image, ctr, 4, circlecolor, -1)

            # Print the ArUco marker ID on the image
            cv2.putText(self.image, str(markerid), txt,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, textcolor, 2)

            #print("ArUco marker ID:", markerid)
            i = i + 2
            j = j + 1

        # Show the output image
        # cv2.imshow("Processed Image", image)
        # Publish the resulting image (to be viewed by rqt_image_view)
        # self.images_pub.publish(self.bridge.cv2_to_imgmsg(self.image, 'bgr8'))
        # rospy.sleep(0.25)
        # cv2.waitKey(0)
    

    #
    # Determine the XYZ position (in task space) of some detected object
    #
    def find_xyz(self, data):
        # Run the Aruco Detect to set up the board
        self.aruco_detect(data)

        # Close to thrower, far from receiver
        coord = np.array([[0,1],[2,3],[4,5],[6,7]])
        p0 = (-0.055, 0.115)
        for i in [0,1,2,3]:
            if self.marker_id[i] == 0:
                p0 = p0
                a0 = (self.center_values[coord[i,0]], self.center_values[coord[i,1]])
            elif self.marker_id[i] == 1:
                p1 = (p0[0], p0[1] + 0.555)
                a1 = (self.center_values[coord[i,0]], self.center_values[coord[i,1]])
            elif self.marker_id[i] == 2:
                p2 = (p0[0] - 0.555, p0[1])
                a2 = (self.center_values[coord[i,0]], self.center_values[coord[i,1]])
            elif self.marker_id[i] == 3:
                p3 = (p0[0] - 0.555, p0[1] + 0.555)
                a3 = (self.center_values[coord[i,0]], self.center_values[coord[i,1]])
        
        pixels = np.float32([[a0],[a1],[a2],[a3]])
        coords = cv2.undistortPoints(pixels, self.camK, self.camD)
        points = np.float32([p0, p1, p2, p3])
        self.M = cv2.getPerspectiveTransform(coords, points)

        sack_loc = self.sack_detector(data)

        if sack_loc != "No Blob":
            pixels = np.float32([[sack_loc[0],sack_loc[1]]])
            coords = cv2.undistortPoints(pixels, self.camK, self.camD)
            points = cv2.perspectiveTransform(coords, self.M)
            rospy.loginfo(points[0,0,:])
            self.blobpub.publish(points[0,0,:])
            # return points[0,0,:]
        else:
            return sack_loc
            #self.blobpub.publish(sack_loc, points[0,0,:])


    def sack_detector(self, data):
        # Check if the detector should be run (has long enough passed since
        # the last detection?).
        if self.detection_interval is not None \
                and (rospy.Time.now() - self.last_detection).to_sec() < self.detection_interval:
            return

        self.last_detection = rospy.Time.now()

        # Convert ROS Image message to OpenCV image array.
        #img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Convert BGR image to HSV.
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

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
        # self.images_pub.publish(self.bridge.cv2_to_imgmsg(thresh, 'mono8'))

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
            # approx = cv2.approxPolyDP(
            #     contour, 0.01*cv2.arcLength(contour, True), True)
            area = cv2.contourArea(contour)
            if area > 30:
                circular_contours.append(contour)
        if len(circular_contours) != 0:
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
            return detection_msg.x, detection_msg.y
        else:
            stri = "No Blob"
            return stri

    #
    # Start the Detector (not sure if we really need this function)
    #
    def start(self):
        # Start the image subscriber (listen for images from camera node).
        rospy.Subscriber('/usb_cam/image_raw', Image,
                         self.find_xyz)

        

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
    d = Detector(h_lims=(100, 200), s_lims=(100, 230), v_lims=(100, 250))
    d.start()