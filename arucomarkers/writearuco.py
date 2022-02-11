#!/usr/bin/env python3
#
#   writearuco.py
#
#   Generate an ArUco marker (from a given dictionary).
#

# ROS Imports
import cv2


# Set up the ArUco detector.  Use a dictionary with the appropriate
# tags.  DICT_6x6_1000 has 1000 options (way too many).  DICT_6x6_250
# was used to create the markers in the test image (IDs up to 250).
# DICT_6x6_50 is probably enough for our projects...
dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
params = cv2.aruco.DetectorParameters_create()

# Create a marker, ID#23, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict, 23, 200);
cv2.imwrite("marker23.png", markerimage);
