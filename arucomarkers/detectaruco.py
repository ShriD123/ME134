#!/usr/bin/env python3
#
#   detectaruco.py
#
#   Run the ArUco detector on a single image.
#

# ROS Imports
import cv2
#import imutils          # Just for testing/rotating image.


# Grab the image.
image = cv2.imread("original.jpeg")

# Show the original image.
#cv2.imshow("Original Image", image)
#cv2.waitKey(0)

# Rotate the image to make things harder for the detector!
# You do NOT want to take this!
(h, w) = image.shape[:2]
(cX, cY) = (w // 2, h // 2)
M = cv2.getRotationMatrix2D((cX, cY), 45, 1.0)
image = cv2.warpAffine(image, M, (w, h))
#image = imutils.rotate(image, 45)


# Set up the ArUco detector.  Use a dictionary with the appropriate
# tags.  DICT_6x6_1000 has 1000 options (way too many).  DICT_6x6_250
# was used to create the markers in the test image (IDs up to 250).
# DICT_6x6_50 is probably enough for our projects...
dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
params = cv2.aruco.DetectorParameters_create()

# Detect.
(corners, ids, rejected) = cv2.aruco.detectMarkers(image, dict, parameters=params)

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
cv2.imshow("Processed Image", image)
cv2.waitKey(0)
