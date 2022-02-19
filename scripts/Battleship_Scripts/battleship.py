#!/usr/bin/env python3

import rospy
import math
import kinematics as kin
import numpy as np

''' This is the main code for the Cornhole Battleship project. This is where all
the constituent nodes are created and the main loop is run. '''




###############################################################################
#
#  Helper Functions
#
class Helper:
    #TODO
    # This should not be a class, but rather global functions. This is just
    # a placeholder as a reminder to add in helper functions (if necessary).
    pass


###############################################################################
#
#  Battleship Class
#
class Battleship:
    #
    # Initialize.
    #
    def __init__(self):
        
        #TODO: Initialize any important variables

        #TODO: Create all the publishers and subscribers for each node

        #TODO: Create the Visualization, Detector, Thrower, Receiver Nodes

        #TODO: Generate the robot's board and move all arms to a starting position.
        pass
        
    #    
    # Update every 10ms!
    #
    def update(self, t):

        #TODO: Create JointState messages for both arms
        pass
        
    #
    # Callback Function 
    #
    def callback(self, msg):
        #TODO: Figure out number of callback functions and how to implement
       
                            
    

###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('Battle')

    # Instantiate the battleship generator object, encapsulating all
    # the computation and local variables.
    battle = Battleship()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    starttime = rospy.Time.now()
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Update the controller.
        battle.update(t)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()
