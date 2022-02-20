#!/usr/bin/env python3

import rospy
import math
import kinematics as kin
import numpy as np

from detector import Detector
from receiver import Receiver
from thrower import Thrower
from visualize_battle import Visualization
from algorithm import Bayes

''' This is the main code for the Cornhole Battleship project. This is where all
the constituent nodes are created and the main loop is run. '''


###############################################################################
#
#  Helper Functions
#




###############################################################################
#
#  Battleship Class
#
class Battleship:
    #
    # Initialize.
    #
    def __init__(self):
        # Initialize the major constituents
        self.detector = Detector()
        self.receiver = Receiver()
        self.thrower = Thrower()
        self.vis = Visualization()
        self.bayes = Bayes()

        # TODO: Initialize any helpful global variables
        self.curr_t = 0.0
        self.board_size = 5
        self.ship_sizes = [4, 3, 2]
        
        # Initialize the board, saved as a 5x5 numpy array
        # 1 means a ship is at that location, while 0 means it is empty.
        self.board = np.zeros(self.board_size, self.board_size)
        ship_locations = self.find_ships(self.board_size, self.ship_sizes)
        for i in range(len(ship_locations)):
            self.board[ship_locations[i]] = 1

        # Visualize the board.
        self.vis.draw_board(self.board)




        # Create publishers and subscribers for each node.

        # Move all the arms to a starting position.

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
