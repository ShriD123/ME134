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
import tkinter as tk
import tkinter.ttk as ttk
from algorithm import find_ships

'''This code encapsulates the functionality for the visualization aspects that show
the current board and guesses on the monitor and plays sounds for hits or misses.'''

# Turn matplotlib interactive mode on
plt.ion()

plt.rcParams.update({'font.family':'monospace'})

###############################################################################
# Scoreboard

class Scoreboard(tk.Tk):
    def __init__(self):
        super().__init__()
        # configure the root window
        self.title('BATTLESHIP SCOREBOARD')
        
        self.headerfont = ('Arial', 35)
        self.textfont = ('Arial', 25)

        # label
        self.robotlabel = ttk.Label(self, text='ROBOT', font=self.headerfont)
        self.humanlabel = ttk.Label(self, text='HUMAN', font=self.headerfont)
        
        # Scores
        self.robothits = ttk.Label(self, text='0 Hits', font=self.textfont)
        self.robotmiss = ttk.Label(self, text='0 Miss', font=self.textfont)
        
        self.humanhits = ttk.Label(self, text='0 Hits', font=self.textfont)
        self.humanmiss = ttk.Label(self, text='0 Miss', font=self.textfont)
        
        # Dividing Line
        self.divider =ttk.Separator(self, orient='vertical').grid(column=1, row=0, rowspan=3, sticky='ns')
        
        self.update_GUI()
        
    def update_score(self, hits, miss, player='robot'):
        """ Updates score based on board """
        # Tupdate the scoreboard based on readings
        # NOTE: Robot hits/miss shows hits on human board; thus why these are swapped
        if player == 'human':
            self.robothits.config(text='{} Hits'.format(hits))
            self.robotmiss.config(text='{} Miss'.format(miss))
        elif player == 'robot':
            self.humanhits.config(text='{} Hits'.format(hits))
            self.humanmiss.config(text='{} Miss'.format(miss))
        self.update_GUI()
    
    def declare_winner(self, winner):
        if winner == 'ROBOT':
            self.robotlabel.config(text='ROBOT', font=('Arial', 75), foreground='#ff1944')
            self.humanlabel.config(text='WINS!!!', font=('Arial', 75), foreground='#ff1944')
        elif winner == 'OPPONENT':
            self.robotlabel.config(text='HUMAN', font=('Arial', 75), foreground='#ff1944')
            self.humanlabel.config(text='WINS!!!', font=('Arial', 75), foreground='#ff1944')
        self.update_GUI()
            
    def update_GUI(self):
        self.robotlabel.grid(row=0, column=0, padx=10, pady=10)
        self.humanlabel.grid(row=0, column=2, padx=10, pady=10)
        self.robothits.grid(row=1, column=0)
        self.robotmiss.grid(row=2, column=0)
        self.humanhits.grid(row=1, column=2)
        self.humanmiss.grid(row=2, column=2)

        

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
        self.fig, (self.robot_ax, self.human_ax) = plt.subplots(1, 2)
        
        # Set size of figure
        self.fig.set_size_inches(10, 6)
        
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
        
        # Initialize human board (the board which the human's ships are on, and the robot is trying to hit)
        self.human_ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)
        
        # Initialize robot board (the board which the robot's ships are on, and the human is trying to hit)
        self.robot_ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)

        
        self.draw_grid()
        
        # Draw 
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        
        # Create dictionary to convert A1 notation to (x, y)
        self.cell_lookup = {}
        for i in range(self.M):
            for j in range(self.N):
                rowcol = self.YTICKS[j] + self.XTICKS[i]
                self.cell_lookup[rowcol] = (i, j)
        
        # Initialize the scoreboard
        self.score = Scoreboard()


    ########################################################################    
    # FUNTION TO DRAW BOARD
    ######################################################################## 
    def draw_board(self, board, ships, ship_sizes=[4, 3, 2], player='robot'):
        """
        Board is 5x5 np array of state of board, for example:
        board = np.array([[0, 0, 0, 0, 0], 
            [1, 1, 1, 1, 0],
            [2, 2, 1, 0, 0],
            [3, 3, 0, 0, 1],
            [0, 0, 0, 3, 1]])
        Ship_sizes are sizes of the ships used, for example:
        ship_sizes = [4, 3, 2]
        Ships is list of list of tuples defining (x, y) location of ships, for example:
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
        # TRANSPOSE IF X AND Y AXES ARE FLIPPED
        ax.imshow(color, aspect='equal', interpolation='none',
                extent=[0, self.N, 0, self.M], zorder=0)

        self.draw_grid()
        
        # Update the player's score on the scoreboard
        self.update_scores(board, player=player)
        # Force the figure to pop up.
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        
    # FUNCTION TO PUT A DOT AT DESIRED LOCATION SPECIFIED
    
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
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    #-----------------------------------------------------------------------    
    # HELPER FUNCTIONS FOR draw_board
    #-----------------------------------------------------------------------
    def draw_ships(self, ships, ax):
        """ Helper Function to draw ships """
        # Mark the ships
        for i, ship in enumerate(ships):
            # ship is a list of tuples, e.g. [(1, 2), (1, 3)]
            # Cast into numpy array of form np.array([[1, 2], [1, 3]])
            ship_array = np.array(ship)
            # Find corners of ship
            # SWITCH THESE IF X AND Y AXES ARE FLIPPED
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
            rect = patches.Rectangle((minx, miny), xlength, ylength, linewidth=15, edgecolor=shipcolor, facecolor='none')
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
        
        # Main title
        self.fig.suptitle('BATTLESHIP', fontsize=20)

        
    ########################################################################    
    # FUNTION TO ALLOW USER TO CHOOSE SHIP POSITIONS
    ######################################################################## 
    def choose_ship_position(self):
        """ This function enables the user to choose where to place ships
        NOTE: THIS IS A BLOCKING FUNCTION THAT BLOCKS UNTIL USER IS DONE PICKING SHIPS, NOW DEPRECATED
        This creates a GUI that allows the user to position their ships
        Returns user chosen positions after user clicks 'done'
        in list of tuple form [[(x11, y11), (x12, y12), (x13, y13)], [(x21, y21), (x22, y22)], and so on]
        """
        sg.theme('Default1')
        # Layout of the GUI is defined here   

        layout = [[sg.Frame('Ship to Move', [[sg.Radio('Battleship (4)', 'ship_sel', key='Battleship', default=True)],
                                          [sg.Radio('Cruiser (3)', 'ship_sel', key='Cruiser', default=False)],
                                          [sg.Radio('Destroyer (2)', 'ship_sel', key='Destroyer', default=False)]],
                                          element_justification='left'),
                  sg.Frame('Move Ship', [[sg.Button('UP', key='Up')],
                                          [sg.Button('L', key='Left'), sg.Button('R', key='Right')],
                                          [sg.Button('DN', key='Down')],
                                          [sg.Button('CCW', key='Counterclockwise'), sg.Button('CW', key='Clockwise')]],
                                          element_justification='center')],
                  [sg.Button('Done')]]      

        window = sg.Window('CONFIGURE YOUR SHIPS', layout)      

        ship_sizes = [4, 3, 2]
        # Locations of the three ships
        battleship = [(0, 0), (1, 0), (2, 0), (3, 0)]
        cruiser = [(1, 2), (2, 2), (3, 2)]
        destroyer = [(4, 3), (4, 4)]
        ships = [battleship, cruiser, destroyer]
        
        # Draw the board
        self.draw_board(np.zeros((5, 5)), ships, player='human') 

        while True:                             
            event, values = window.read() 
            print(event, values)       
            if event == sg.WIN_CLOSED or event == 'Done':
                break      
            # If we want to move the ship
            if event in ['Up', 'Left', 'Right', 'Down', 'Clockwise', 'Counterclockwise']:
                # Create copy of ships that we will modify
                newships = copy.deepcopy(ships)
                # Which ship we want to move, based on radio button selection
                if   values['Battleship'] == True:
                    newships[0] = self.move_ship(newships[0], move=event)
                elif values['Cruiser'] == True:
                    newships[1] = self.move_ship(newships[1], move=event)
                elif values['Destroyer'] == True:
                    newships[2] = self.move_ship(newships[2], move=event)
                self.draw_board(np.zeros((5, 5)), newships, player='human') 
                time.sleep(0.05)
                # Check validity of new configuration
                if self.check_ship_validity(newships):
                    # If the new configuration with the moved ships is valid, change ships to newships
                    ships = newships
                # Plot the ship locations
                self.draw_board(np.zeros((5, 5)), ships, player='human')          

        window.close()
        return ships
    
    def choose_ship_position_rand(self):
        """NOTE: THIS IS A BLOCKING FUNCTION
            Displays the ship configuration given, and a prompt asking if user is ok with this configuration
            Returns True if user clicks Yes, otherwise returns False
        """
        ship_sizes = [4, 3, 2]
        ships = find_ships(self.M, ship_sizes)

        self.draw_board(np.zeros((5, 5)), ships, player='human')
        
        sg.theme('Default1')
        # Layout of the GUI is defined here   

        layout = [[sg.Text('A random configuration has been generated for you.')],
        [sg.Frame('Configure Board', [[sg.Button('Use This Configuration', key='yes'), sg.Button('Generate another Configuration', key='no')]])]]      

        window = sg.Window('CONFIGURE YOUR SHIPS', layout)      
        
        userok = False

        while not userok:                             
            event, values = window.read() 
            print(event, values)       
            if event == sg.WIN_CLOSED:
                break  
            if event == 'yes':
                userok = True
                break
            if event == 'no':
                ships = find_ships(self.M, ship_sizes)
                self.draw_board(np.zeros((5, 5)), ships, player='human')
        window.close()
        
        return ships
        
    #-----------------------------------------------------------------------    
    # HELPER FUNCTIONS FOR choose_ship_position
    #-----------------------------------------------------------------------
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
        elif move in ['Counterclockwise', 'Clockwise']:
            # Want to rotate about lower left corner, so shift to origin first
            (minx, miny) = (np.amin(newship_arr[:, 0]), np.amin(newship_arr[:, 1]))
            newship_arr -= np.array((minx, miny))
            if move == 'Counterclockwise':
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
            # Shift back to position
            newship_arr += np.array((minx, miny))
        
        # Convert numpy array back to list of tuples
        newship = []
        for loc in newship_arr:
            newship.append(tuple(loc.flatten()))
        # Return new ship locations
        return newship           
    
    def check_ship_validity(self, ships):
        """ Helper function to check whether list of ships are valid or not """
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
        # Check if there are any overlapping points between ships (points that are the same)
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
    def make_sound(self, hit=False):
    #     #TODO: Download the hit or miss audio & insert path
        # Need to set False in playsound so that it will not block
        path = '/home/me134/me134ws/src/ME134/'
        hitpath = path + 'sounds/hit.mp3'
        misspath = path + 'sounds/miss.wav'
        if hit:
            playsound(hitpath, False)
        else:
            playsound(misspath, False)
            
     
    # Declare Winner
    def declare_winner(self, winner):
        """ Winner either ROBOT or OPPONENT """
        self.score.declare_winner(winner)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def update_scores(self, board, player='robot'):
        hit_value = np.ones((5, 5)) * self.HIT
        miss_value = np.ones((5, 5)) * self.MISS
        hits = np.count_nonzero(np.isclose(board, hit_value))
        miss = np.count_nonzero(np.isclose(board, miss_value))
        self.score.update_score(hits, miss, player=player)
        

    


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
    


    human_ships = vis.choose_ship_position_rand()
    print(human_ships)
    vis.draw_board(board, ships, player='robot')
    print('drawing board')
    
    time.sleep(0.5)
    vis.make_sound(hit=True)
    time.sleep(3.0)
    vis.make_sound(hit=False)
    
    vis.draw_nextmove(np.array([[1, 2]]), player='robot')
    print('drawing robot next move')
    time.sleep(0.5)
    
    vis.draw_board(board, human_ships, player='human')
    print('drawing human')
    vis.declare_winner('ROBOT')
    time.sleep(2)
    
    vis.declare_winner('OPPONENT')
    time.sleep(2)
    #while True:
    #    vis.draw_board(np.random.randint(5, size=(5, 5)), ships, ship_sizes, player = 'robot')
        
    #    vis.draw_board(np.random.randint(5, size=(5, 5)), ships, ship_sizes, player='human')
    
    

