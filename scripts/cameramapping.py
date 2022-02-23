#!/usr/bin/env python3
#
#   cameramapping.py
#
#   Demo how to un-distort and map pixel values.
#
#   Incoming:       /usb_cam/camera_info   Source image info
#                   /usb_cam/image_raw     Source image
#
#   Publishers:     /detector/image_raw    Debug image
#

# ROS Imports
import rospy
import cv2
import cv_bridge
import numpy as np

from sensor_msgs.msg  import Image
from sensor_msgs.msg  import CameraInfo


#
#  Detector Node Class
#
class Detector:
    def __init__(self):
        # Grab an instance of the camera data.
        rospy.loginfo("Waiting for camera info...")
        msg = rospy.wait_for_message('/usb_cam/camera_info', CameraInfo);
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

    def test_undistort(self):
        # Do lots of processing to get some pixel values.
        uv1 = (300.0, 240.0)
        uv2 = (340.0, 250.0)

        # OpenCV wants a MxN array of (u,v) points, so that the list
        # is actually a MxNx2 numpy array.  We arrange the points as
        # an Mx1 list, so get Mx1x2.
        # pixels = np.float32([uv1, uv2]).reshape((-1,1,2))
        pixels = np.float32([[uv1], [uv2]])

        # Undistort: turn pixel values into "normalized image coordinates"
        coords = cv2.undistortPoints(pixels, self.camK, self.camD)

        # Report the full list.
        for (uv, c) in zip(pixels.reshape((-1,2)), coords.reshape((-1,2))):
            print("Converted ", uv, " -> ", c)

        # Extract.  Again note OpenCV has the points in an MxNx2
        # array.  Of which we arranged things as Mx1x2...
        c1 = coords[0,0,:]
        c2 = coords[1,0,:]

        print("Point 1:  pixels ", uv1, " = image coords ", c1)
        print("Point 2:  pixels ", uv2, " = image coords ", c2)

    def test_getTransform(self):
        # Pick 4 table locations, let's just put them in a 2m square.
        # I presume you would measure the locations of 4 ArUco
        # markers.
        p1 = (0.0, 0.0)
        p2 = (2.0, 0.0)
        p3 = (0.0, 2.0)
        p4 = (2.0, 2.0)

        # Pick 4 matching normalized image coordinates.  I presume
        # these would come from the AruCo detector.  Then undistorted
        # as above.
        c1 = (-0.5, -0.5)
        c2 = ( 0.5, -0.35)
        c3 = (-0.5,  0.5)
        c4 = ( 0.5,  0.5)

        # Create the lists.  Just 4 element of (x.y) each.
        coords = np.float32([c1, c2, c3, c4])
        points = np.float32([p1, p2, p3, p4])

        # Create the transform.
        self.M = cv2.getPerspectiveTransform(coords, points)

        # Report.
        print("Normalized Image Coordinates:\n", coords)
        print("Table Points:\n", points)
        print("Perspective transform: \n", self.M)

    def pixel2table(self, uv):
        pixels = np.float32([[uv]])
        coords = cv2.undistortPoints(pixels, self.camK, self.camD)
        points = cv2.perspectiveTransform(coords, self.M)
        return points[0,0,:]

    def test_pixel2table(self):
        # Assume the detector has provided some pixel values:
        uv = (200.0, 100.0)

        # Map to the table.
        p  = self.pixel2table(uv)

        print("Point:  pixels ", uv, " = table coords ", p)
        
        
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.  You can override the name using the
    # 'rosrun .... __name:=something' convention.
    rospy.init_node('cameramapping')

    # Instantiate the Detector object.
    detector = Detector()

    # Run a test cases.
    detector.test_undistort()
    detector.test_getTransform()
    detector.test_pixel2table()


    # Continually process until shutdown.
    #rospy.loginfo("Continually processing latest pending images...")
    #rospy.spin()

    # Report completion.
    rospy.loginfo("Done!")
