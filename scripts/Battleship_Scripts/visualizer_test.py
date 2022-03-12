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
import visualize_battle as vis

'''This code encapsulates the functionality for the visualization aspects that show
the current board and guesses on the monitor and plays sounds for hits or misses.'''


import PySimpleGUI as sg      

sg.theme('Default1')    # Keep things interesting for your users

layout = [[sg.Button('Draw Robot')],
           [sg.Button('Draw Human')],
            [sg.Button('Do something else')]]      

window = sg.Window('Window that stays open', layout)      

visualize = vis.Visualizer()
board = np.array([[0, 0, 0, 0, 0], 
                                [1, 1, 1, 1, 0],
                                [2, 2, 1, 0, 0],
                                [3, 3, 0, 0, 1],
                                [0, 0, 0, 3, 1]])

ship_sizes = [4, 3, 2]
ships = [[(0, 0), (1, 0), (2, 0), (3, 0)], [(1, 1), (2, 1), (3, 1)], [(4, 3), (4, 4)]]

while True:                             # The Event Loop
    event, values = window.read() 
    print(event, values)       
    if event == sg.WIN_CLOSED or event == 'Exit':
        break      
    elif event == 'Draw Robot':
        board = np.random.randint(5, size=(5, 5))
        visualize.draw_board(board, ships, ship_sizes, player='robot')
    elif event == 'Draw Human':
        board = np.random.randint(5, size=(5, 5))
        visualize.draw_board(board, ships, ship_sizes, player='human')
    elif event == 'Do something else':
        print('Doing something else')

window.close()

    

