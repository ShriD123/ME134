#!/usr/bin/env python3

import rospy
import math
#import kinematics as kin
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
from playsound import playsound

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
        # Close the old figure.
        plt.close()
        
        # Initialize any important variables
        self.M = 5
        self.N = 5

        #Create all the necessary global parameters for the visualization
        self.fig = plt.figure()
        self.ax = plt.axes()
        self.XTICKS = ['1', '2', '3', '4', '5']
        self.TICKS_POS = [0.5, 1.5, 2.5, 3.5, 4.5]
        self.YTICKS = ['A', 'B', 'C', 'D', 'E']

        self.WATER = 0
        self.SHIP = 1
        self.HIT = 2
        self.MISS = 3
        self.SUNK = 4
        
        # Show a blank display
        color = np.ones((self.M,self.N,3))
        self.ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)

        # Force the figure to pop up.
        plt.xticks(self.TICKS_POS, self.XTICKS)
        plt.yticks(self.TICKS_POS, self.YTICKS)
        minor_locator = AutoMinorLocator(2)
        plt.gca().xaxis.set_minor_locator(minor_locator)
        plt.gca().yaxis.set_minor_locator(minor_locator)
        plt.grid(which='minor')
        plt.pause(1)


    #    
    # Draw Board
    #
    def draw_board(self, ships):

        # Draw the grid, zorder 1 means draw after zorder 0 elements.
        for m in range(self.M+1):
            self.ax.axhline(m, lw=1, color='b', zorder=1)
        for n in range(self.N+1):
            self.ax.axvline(n, lw=1, color='b', zorder=1)

        # Create the color range. 
        color = np.ones((self.M,self.N,3))
        for m in range(self.M):
            for n in range(self.N):
                if   ships[m,n] == self.WATER:
                    color[m,n,0:3] = np.array([1.0, 1.0, 1.0])   # White
                elif ships[m,n] == self.SHIP:
                    color[m,n,0:3] = np.array([1.0, 1.0, 0.0])   # Yellow
                elif ships[m,n] == self.HIT:
                    color[m,n,0:3] = np.array([0.0, 1.0, 0.0])   # Green
                elif ships[m,n] == self.MISS:
                    color[m,n,0:3] = np.array([0.0, 0.0, 1.0])   # Blue
                elif ships[m,n] == self.SUNK:
                    color[m,n,0:3] = np.array([1.0, 0.0, 0.0])   # Red
        
        # Draw the boxes
        self.ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)

        # Force the figure to pop up.
        plt.pause(1)


    #    
    # Make Sounds of Hit/Miss
    #
    def make_sound(self, hit=False):
        #TODO: Download the hit or miss audio & insert path
        if hit:
            playsound('hit.mp3')
        else:
            playsound('miss.mp3')


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    vis = Visualizer()

    ships = np.array([[0, 0, 0, 0, 0], 
            [1, 1, 1, 1, 0],
            [2, 2, 1, 0, 0],
            [3, 3, 0, 0, 1],
            [0, 0, 0, 3, 1]])

    print(ships.shape)
    
    vis.draw_board(ships)

