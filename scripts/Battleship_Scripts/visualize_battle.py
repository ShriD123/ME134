#!/usr/bin/env python3

import rospy
import math
#import kinematics as kin
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator
from playsound import playsound
import time
import PySimpleGUI as sg
import copy

'''This code encapsulates the functionality for the visualization aspects that show
the current board and guesses on the monitor and plays sounds for hits or misses.'''

# Turn matplotlib interactive mode on
plt.ion()


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
        self.fig, (self.human_ax, self.robot_ax) = plt.subplots(1, 2)
        
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
        
        # Initialize human board
        self.human_ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)
        
        # Initialize robot board
        self.robot_ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)

        
        self.draw_grid()
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Create dictionary to convert A1 notation to (x, y)
        self.cell_lookup = {}
        for i in range(self.M):
            for j in range(self.N):
                rowcol = self.YTICKS[j] + self.XTICKS[i]
                self.cell_lookup[rowcol] = (i, j)
        


    #    
    # Draw Board
    #
    def draw_board(self, board, ships, ship_sizes, player='robot'):
        """
        Board is 5x5 np array of state of board, for example:
        board = np.array([[0, 0, 0, 0, 0], 
            [1, 1, 1, 1, 0],
            [2, 2, 1, 0, 0],
            [3, 3, 0, 0, 1],
            [0, 0, 0, 3, 1]])
        Ship_sizes are sizes of the ships used, for example:
        ship_sizes = [4, 3, 2]
        Ships is list of tuples defining (x, y) location of ships, for example:
        ships = [[(0, 3), (1, 3), (2, 3), (3, 3)], [(0, 2), (1, 2), (2, 2)], [(4, 0), (4, 1)]]
        """
        
        # Which player do we want to draw the board for
        if player == 'robot':
            # Use the robot subplot axes and the robot ships stored
            ax = self.robot_ax
        else:
            # Use the human subplot axes and the robot ships stored
            ax = self.human_ax
            
        # Clear previous stuff
        ax.clear()


        # Create the color range. 
        color = np.ones((self.M,self.N,3))
        for m in range(self.M):
            for n in range(self.N):
                if   board[m,n] == self.WATER:
                    color[m,n,0:3] = np.array([1.0, 1.0, 1.0])   # White
                elif board[m,n] == self.SHIP:
                    color[m,n,0:3] = np.array([1.0, 1.0, 0.0])   # Yellow
                elif board[m,n] == self.HIT:
                    color[m,n,0:3] = np.array([0.0, 1.0, 0.0])   # Green
                elif board[m,n] == self.MISS:
                    color[m,n,0:3] = np.array([0.0, 0.0, 1.0])   # Blue
                elif board[m,n] == self.SUNK:
                    color[m,n,0:3] = np.array([1.0, 0.0, 0.0])   # Red
        
        # Mark the ships
        self.draw_ships(ships, ax)
        
        # Draw the boxes
        ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)

        self.draw_grid()
        
        # Force the figure to pop up.
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        

        
    # Mark Position
    
    def draw_nextmove(self, loc, player='robot'):
        """ Draws X at position specified
        loc = np.array([[X location, Y location]])
        """
        # Which player do we want to draw the board for
        if player == 'robot':
            ax = self.robot_ax
        else:
            ax = self.human_ax
            
        ax.scatter(loc.flatten()[0] + 0.5, loc.flatten()[1] + 0.5, s=300, c='r', marker='o')
        self.draw_grid()
        
        # Force the figure to pop up
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def draw_ships(self, ships, ax):
        """ Helper Function to draw ships """
        # Mark the ships
        for i, ship in enumerate(ships):
            # ship is a list of tuples, e.g. [(1, 2), (1, 3)]
            # Cast into numpy array of form np.array([[1, 2], [1, 3]])
            ship_array = np.array(ship)
            # Find corners of ship
            
            (minx, miny) = (np.amin(ship_array[:, 0]), np.amin(ship_array[:, 1]))
            (maxx, maxy) = (np.amax(ship_array[:, 0]), np.amax(ship_array[:, 1]))
            
            # Size of rectangle to draw
            xlength = maxx - minx + 1
            ylength = maxy - miny + 1
            
            shipsize = len(ship)
            
            #  Different colors to represent different ship sizes
            if shipsize == 4:
                shipcolor = 'tab:blue'
            elif shipsize == 3:
                shipcolor = 'tab:orange'
            elif shipsize == 2:
                shipcolor = 'tab:brown'
            else:
                shipcolor = 'k'
                
            # Outline of ship rectangle
            rect = patches.Rectangle((minx, miny), xlength, ylength, linewidth=5, edgecolor=shipcolor, facecolor='none')
            ax.add_patch(rect)
            rect.set_clip_path(rect)
            # Fill for ship rectangle
            rectfill = patches.Rectangle((minx, miny), xlength, ylength, edgecolor='none', facecolor=shipcolor, alpha=0.1)
            ax.add_patch(rectfill)
    
        

    def draw_grid(self):
        """ Helper Function to draw grid and titles on plot """
        # Set ticks and labels for human and robot
        plt.setp((self.human_ax, self.robot_ax), xticks=self.TICKS_POS, xticklabels=self.XTICKS,
                  yticks=self.TICKS_POS, yticklabels=self.YTICKS)
        minor_locator = AutoMinorLocator(2)
        # Create grid for human
        self.human_ax.xaxis.set_minor_locator(minor_locator)
        self.human_ax.yaxis.set_minor_locator(minor_locator)
        self.human_ax.grid(which='minor', zorder=1.0)
        self.human_ax.set_title('Human')
        
        # Create grid for robot
        self.robot_ax.xaxis.set_minor_locator(minor_locator)
        self.robot_ax.yaxis.set_minor_locator(minor_locator)
        self.robot_ax.grid(which='minor', zorder=1.0)
        self.robot_ax.set_title('Robot')

        
    
    def choose_ship_position(self):
        """ This function enables the user to choose where to place ships
        NOTE: THIS IS A BLOCKING FUNCTION
        """
        sg.theme('Default1')    # Keep things interesting for your users

        layout = [[sg.Frame('Ship to Move', [[sg.Radio('Battleship', 'ship_sel', default=True)],
                                          [sg.Radio('Cruiser', 'ship_sel', default=False)],
                                          [sg.Radio('Destroyer', 'ship_sel', default=False)]])],
                  [sg.Frame('Move Ship', [[sg.Button('Up')],
                                          [sg.Button('Left'), sg.Button('Right')],
                                          [sg.Button('Down')],
                                          [sg.Button('Counterclockwise'), sg.Button('Clockwise')]])],
                  [sg.Button('Done')]]      

        window = sg.Window('Window that stays open', layout)      

        ship_sizes = [4, 3, 2]
        # Locations of the three ships
        battleship = [[(0, 0), (1, 0), (2, 0), (3, 0)]]
        cruiser = [(1, 1), (2, 1), (3, 1)]
        destroyer = [(4, 3), (4, 4)]
        ships = [battleship, cruiser, destroyer]

        while True:                             
            event, values = window.read() 
            print(event, values)       
            if event == sg.WIN_CLOSED or event == 'Done':
                break      
            # If we want to move the ship
            if event in ['Up', 'Left', 'Right', 'Down', 'Clockwise', 'Counterclockwise']:
                # Create copy of ships
                newship = copy.deepcopy(ships)
                if values['Battleship'] == True:
                    newships[0] = self.move_ship(newships[0], move=event)
                elif values['Cruiser'] == True:
                    newships[1] = self.move_ship(newships[1], move=event)
                elif values['Destroyer'] == True:
                    newships[2] = self.move_ship(newships[2], move=event)
                
                    
            elif event == 'Draw Robot':
                board = np.random.randint(5, size=(5, 5))
                visualize.draw_board(board, ships, ship_sizes, player='robot')
            elif event == 'Draw Human':
                board = np.random.randint(5, size=(5, 5))
                visualize.draw_board(board, ships, ship_sizes, player='human')
            elif event == 'Do something else':
                print('Doing something else')

        window.close()
    
    def move_ship(self, ship, move='Up'):
        """ Helper function to move ship"""
        # Convert to np array for operations
        newship_arr = np.array(ship)
        # Translate by adding/subtracting to x or y coordinate
        if move == 'Up':
            newship_arr[:, 1] += 1
        elif move == 'Down':
            newship_arr[:, 1] -= 1
        elif move == 'Left':
            newship_arr[:, 0] -= 1
        elif move == 'Right':
            newship_arr[:, 0] += 1
        elif move == 'Counterclockwise':
            # To rotate counterclockwise, (x, y) = (-y, x)
            # Swap x and y
            newship_arr[:, [1, 0]] = newship_arr[:, [0, 1]]
            # Negative the x position
            newship_arr[:, 0] = -newship_arr[:, 0]
        elif move == 'Clockwise':
            # To rotate clockwise, (x, y) = (y, -x)
            # Swap x and y
            newship_arr[:, [1, 0]] = newship_arr[:, [0, 1]]
            # Negative the y position
            newship_arr[:, 1] = -newship_arr[:, 1]
        
        # Convert numpy array back to list of tuples
        newship = []
        for loc in newship_arr:
            newship.append(tuple(loc.flatten()))
        # Return new ship locations
        return newship
            
    
    def check_ship_validity(self, ships):
        """ Helper function to check whether ships are valid or not """
        # ships is a list of list of tuples, e.g. [(1, 2), (1, 3)]
        for ship in ships:
            ship_array = np.array(ship)
            # Check whether ship is out of bound of board
            out_of_bound = np.any((ship_array < 0)|(ship_array > 4))
            # Check whether ship is horizontal or vertical
            orr_incorrect = not ((np.isclose(np.amax(ship_array[:, 0]), np.amin(ship_array[:, 0])) or 
                          np.isclose(np.amax(ship_array[:, 1]), np.amin(ship_array[:, 1]))))
            # IF any of the ships are out of bound or not horizontal or vertical, exit and return False
            if out_of_bound:
                print('Out of Bound')
                return False
            if orr_incorrect:
                print('Orientation Incorrect')
                return False
        # Flatten ship list to check values of tuples
        
        flattened_ships = [item for sublist in ships for item in sublist]
        seen = []
        # Check if there are any overlapping points (points that are the same)
        for location in flattened_ships:
            # IF any of the locations is the same as the the ones already iterated over, exit and return false
            # Round to nearest integer for consistency
            location = (int(round(location[0])), int(round(location[1])))
            if location in seen:
                print('Overlapping Ships')
                return False
            else:
                seen.append(location)
        
        # If nothing has triggered the function to return False, return True
        return True
        
        
    #    
    # Make Sounds of Hit/Miss
    #
    # def make_sound(self, hit=False):
    #     #TODO: Download the hit or miss audio & insert path
    #     if hit:
    #         playsound('hit.mp3')
    #     else:
    #         playsound('miss.mp3')


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    vis = Visualizer()
    print('initializing object')
    time.sleep(3)

    board = np.array([[0, 0, 0, 0, 0], 
            [1, 1, 1, 1, 0],
            [2, 2, 1, 0, 0],
            [3, 3, 0, 0, 1],
            [0, 0, 0, 3, 1]])

    ship_sizes = [4, 3, 2]
    ships = [[(0, 3), (1, 3), (2, 3), (3, 3)], [(0, 2), (1, 2), (2, 2)], [(4, 0), (4, 1)]]

    
    vis.draw_board(board, ships, ship_sizes, player='robot')
    print('drawing board')
    
    time.sleep(0.5)
    
    vis.draw_nextmove(np.array([[1, 2]]), player='robot')
    print('drawing robot next move')
    time.sleep(0.5)
    
    vis.draw_board(board, ships, ship_sizes, player='human')
    print('drawing human')
    time.sleep(0.5)
    
    #while True:
    #    vis.draw_board(np.random.randint(5, size=(5, 5)), ships, ship_sizes, player = 'robot')
        
    #    vis.draw_board(np.random.randint(5, size=(5, 5)), ships, ship_sizes, player='human')
    
    

