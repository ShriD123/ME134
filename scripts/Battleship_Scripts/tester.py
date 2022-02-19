#!/usr/bin/env python3

import rospy
import math
import kinematics as kin
import numpy as np

''' This is the code for testing each of the individual nodes. Use this for debugging purposes.'''


###############################################################################
#
#  Debugging Functions
#

#    
# Debug the Detector Code
#
def test_detector():
    #TODO: Instantiate a detector object, start it, and see if the output is expected.
    # May want to use RQT for this
    pass


#    
# Debug the Receiver Code
#
def test_receiver():
    #TODO: Instantiate a receiver object, test it, and see if the output is expected.
    # May want to use RVIZ for this
    pass


#    
# Test the Receiver Code on the Arm
#
def test_receiver_IRL():
    #TODO: Instantiate a receiver object, test it, and see if the output is expected.
    # May want to have multiple people to make sure the arm doesn't go crazy
    # Also will want to ensure it works smoothly and is not buggy in RVIZ first.
    pass


#    
# Debug the Thrower Code
#
def test_thrower():
    #TODO: Instantiate a thrower object, test it, and see if the output is expected.
    # May want to use RVIZ for this, then consider a separate function for testing IRL
    pass

#    
# Test the Thrower Code on the Arm
#
def test_thrower_IRL():
    #TODO: Instantiate a thrower object, test it, and see if the output is expected.
    # May want to have multiple people to make sure the arm doesn't go crazy
    # Also will want to ensure it works smoothly and is not buggy in RVIZ first.
    pass


#    
# Debug the Visualization Code
#
def test_visualization():
    #TODO: Instantiate a visualization object, test it, and see if the output is expected.
    # May want to use a separate screen (monitor) for this
    pass


#    
# Debug the Main Code
#
def test_main():
    #TODO: Instantiate a Battleship object, start it, and see if the output is expected.
    # May want to use predefined values and/or printing values to understand what's going on.
    pass


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    # # Uncomment to run the corresponding testing function.
    # test_main()
    # test_visualization()
    # test_detector()
    # test_receiver()
    # test_receiver_IRL()
    # test_thrower()
    # test_thrower_IRL()
    pass
