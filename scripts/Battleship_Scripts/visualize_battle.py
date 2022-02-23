#!/usr/bin/env python3

import rospy
import math
import kinematics as kin
import numpy as np

'''This code encapsulates the functionality for the visualization aspects that show
the current board and guesses on the monitor and plays sounds for hits or misses.'''


###############################################################################
#
#  Generator Class
#
class Visualizer:
    #
    # Initialize.
    #
    def __init__(self):
        
        #TODO: Initialize any important variables

        #TODO: Create all the necessary global parameters for the visualization

        #TODO: Show a blank display (or the initial state)
        pass

    #    
    # Draw Board
    #
    def draw_board(self):
        #TODO: Draw the grid with the positions of the ships on the board.
        # Might want to use Matplotlib for this (but maybe another package will work better)
        #TODO: Relate the probability map to the corresponding colors
        pass


    #    
    # Make Sounds of Hit/Miss
    #
    def make_sound(self):
        #TODO: Make the corresponding sound of hit or miss come from a speaker.
        # Will have to look into how exactly this can work.
        pass
        
    #
    # Update every 10ms!
    #
    def update(self, t):
        #TODO: Update the board state on the screen if something has changed.
        # Not sure if this function is wholly necessary


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    #TODO: Implement this later and see exactly what needs to be in the main code.
    pass
