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

#
# Determine the ship board positions
#
def find_ships(self, board_size, ship_sizes):
    # Eventually return the list of indices corresponding to the ship
    idx = []
    VERTICAL = 0
    HORIZONTAL = 1
    ship_counter = 1

    while ship_counter != len(ship_sizes):
        loc_x = np.floor(np.random.uniform(0, board_size))
        loc_y = np.floor(np.random.uniform(0, board_size))
        orn = np.floor(np.random.uniform(0, 2))  
        ship_found = False

        this_ship = []
        for i in range(ship_sizes(ship_counter)):
            count_iter = 0
            while not space_found:
                # Choose the next position for the ship
                if orn == VERTICAL:
                    if (loc_y + i) < board_size:
                        loc_x_next = loc_x
                        loc_y_next = loc_y + i
                    else:
                        loc_x_next = loc_x
                        loc_y_next = loc_y - i 
                elif orn == HORIZONTAL:
                    if (loc_x + i) < board_size:
                        loc_x_next = loc_x + i
                        loc_y_next = loc_y 
                    else:
                        loc_x_next = loc_x - i
                        loc_y_next = loc_y 

                # Test if the space is already occupied
                if (loc_y_next, loc_x_next) not in idx:
                    # Note: x corresponds to cols and y corresponds to rows
                    this_ship.append((loc_y_next, loc_x_next))
                    space_found = True

                count_iter += 1
                # Test if too many iterations occur (then it's impossible to place ship, so need new loc) 
                if count_iter >= 50:
                    break

        idx.append(this_ship)
        ship_counter += 1



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
