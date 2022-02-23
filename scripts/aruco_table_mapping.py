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
from ME134.msg import array


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
        #self.test_undistort()
        

    def test_undistort(self,data):
        rospy.loginfo("PP")
        # Do lots of processing to get some pixel values.
                # Close to thrower, far from receiver
        coord = np.array([[0,1],[2,3],[4,5],[6,7]])
        for i in [0,1,2,3]:
            if data.marker_id[i] == 0:
                p0 = (0, 0)
                a0 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Far from both
            elif data.marker_id[i] == 1:
                p1 = (0, 0.555)
                a1 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Close to both
            elif data.marker_id[i] == 2:
                p2 = (0.555, 0)
                a2 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Close to receiver, far from thrower
            elif data.marker_id[i] == 3:
                p3 = (0.555, 0.555)
                a3 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])


        # OpenCV wants a MxN array of (u,v) points, so that the list
        # is actually a MxNx2 numpy array.  We arrange the points as
        # an Mx1 list, so get Mx1x2.
        # pixels = np.float32([uv1, uv2]).reshape((-1,1,2))
        pixels = np.float32([[a0], [a1],[a2],[a3]])
        print(pixels)

        # Undistort: turn pixel values into "normalized image coordinates"
        coords = cv2.undistortPoints(pixels, self.camK, self.camD)

        # Report the full list.
        for (uv, c) in zip(pixels.reshape((-1,2)), coords.reshape((-1,2))):
            print("Converted ", uv, " -> ", c)

        # Extract.  Again note OpenCV has the points in an MxNx2
        # array.  Of which we arranged things as Mx1x2...
        c0 = coords[0,0,:]
        c1 = coords[1,0,:]
        c2 = coords[2,0,:]
        c3 = coords[3,0,:]

        print("Point 0:  pixels ", a0, " = image coords ", c0)
        print("Point 1:  pixels ", a1, " = image coords ", c1)
        print("Point 2:  pixels ", a2, " = image coords ", c2)
        print("Point 3:  pixels ", a3, " = image coords ", c3)

    def test_getTransform(self,data):

        # Close to thrower, far from receiver
        coord = np.array([[0,1],[2,3],[4,5],[6,7],[8,9]])
        for i in [0,1,2,3,4]:
            if data.marker_id[i] == 0:
                p0 = (0, 0)
                a0 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Far from both
            elif data.marker_id[i] == 1:
                p1 = (0, 0.555)
                a1 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Close to both
            elif data.marker_id[i] == 2:
                p2 = (0.555, 0)
                a2 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Close to receiver, far from thrower
            elif data.marker_id[i] == 3:
                p3 = (0.555, 0.555)
                a3 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            elif data.marker_id[i] == 4:
                test_point = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
        
        pixels = np.float32([[a0],[a1],[a2],[a3]])
        coords = cv2.undistortPoints(pixels, self.camK, self.camD)
        points = np.float32([p0, p1, p2, p3])
        self.M = cv2.getPerspectiveTransform(coords, points)

        # Report.
        print("Normalized Image Coordinates:\n", coords)
        print("Table Points:\n", points)
        print("Perspective transform: \n", self.M)
        print("test point:")
        print(test_point)
        point = self.pixel2table(test_point)
        print("point = ")
        print(point)

    def pixel2table(self,test_point):
        pixels = np.float32([[test_point[0],test_point[1]]])
        coords = cv2.undistortPoints(pixels, self.camK, self.camD)
        points = cv2.perspectiveTransform(coords, self.M)
        return points[0,0,:]

    def test_pixel2table(self,data):
        # Assume the detector has provided some pixel values:
        #uv = (200.0, 100.0)

        # Close to thrower, far from receiver
        coord = np.array([[0,1],[2,3],[4,5],[6,7]])
        for i in [0,1,2,3]:
            if data.marker_id[i] == 0:
                p0 = (0, 0)
                a0 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Far from both
            elif data.marker_id[i] == 1:
                p1 = (0, 0.555)
                a1 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Close to both
            elif data.marker_id[i] == 2:
                p2 = (0.555, 0)
                a2 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])
            # Close to receiver, far from thrower
            elif data.marker_id[i] == 3:
                p3 = (0.555, 0.555)
                a3 = (data.center_values[coord[i,0]], data.center_values[coord[i,1]])

        points = np.float32([p0, p1, p2, p3])
        # Map to the table.
        p  = self.pixel2table(a0,a1,a2,a3,points)

        print("Point:  pixels ", [a0,a1,a2,a3], " = table coords ", p)
        
        
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.  You can override the name using the
    # 'rosrun .... __name:=something' convention.
    rospy.init_node('cameramapping')

    # Instantiate the Detector object.
    detector = Detector()
    rospy.Subscriber('aruco_center', array, detector.test_getTransform)
    # Run a test cases.
    #detector.test_undistort()
    #detector.test_getTransform()
    #detector.test_pixel2table()
    #detector.pixel2table()


    # Continually process until shutdown.
    rospy.loginfo("Continually processing latest pending images...")
    rospy.spin()

    # Report completion.
    rospy.loginfo("Done!")
