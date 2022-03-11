#!/usr/bin/env python3

import rospy
import math
import numpy as np
#from playsound import playsound

'''This code encapsulates the board checks and calculations, as well as determining
the best next move for the thrower arm to hit the opponent's grid.'''


###############################################################################
#
#  Board Algorithm Class
#
class Board:
    #
    # Initialize.
    #
    def __init__(self, ship_sizes, board_size, aruco_position, robot_board=None, opponent_board=None):
        # Initialize any important variables
        self.M = board_size
        self.N = board_size
        self.ship_sizes = ship_sizes

        self.board_origin = aruco_position      # FIGURE OUT LATER HOW EXACTLY TO INPUT THIS... important for board to xyz and xyz to board fns

        # Constants to discern between different states
        self.WATER = 0
        self.SHIP = 1
        self.HIT = 2
        self.MISS = 3
        self.SUNK = 4

        # Initialize the 5X5 boards for the robot and the opponent
        self.robot = np.zeros((self.M, self.N))
        self.opponent = np.zeros((self.M, self.N))    
        # Keep track of the ships for the robot and the opponent
        self.robot_ships = []        # Should be a list of indices, saving the ships... updated in find_ships
        self.opponent_ships = []        # Should be a list of indices, saving the ships also updated in input_opponent maybe?

        # self.robot = find_ships(board_size, self.ship_sizes)
        # self.opponent = input_opponent(self.ship_sizes, opponent_ship_list)       # Opponent ship list does not exist here

        # Maybe call the input opponent board function here?

        # If we ever need to manually adjust the boards
        if robot_board is not None:
            self.robot = robot_board
        
        if opponent_board is not None:
            self.opponent = opponent_board

    #    
    # Update the robot's board with the corresponding position and result
    #
    def update_robot_board(self, pos, result):
        # Assume result is one of the battleship constants 
        self.robot[pos] = result

        # Check if the whole ship is sunk if it is hit
        if result == self.HIT:

            # Determine which ship that position is in
            ship_loc = None
            for i in self.ship_sizes:
                this_ship = self.robot_ships[i]
                for j in range(i):
                    if pos == this_ship[j]:
                        ship_loc = i

            # Check if all parts of the ship are hit
            hit_ship = self.robot_ships[ship_loc]
            hit_ship_size = self.ship_sizes[ship_loc]
            hit_ship_counter = 0

            for i in range(hit_ship_size):
                if self.robot[hit_ship[i]] == self.HIT:
                    hit_ship_counter += 1

            # If all parts of the ship are hit, change ship to sunk
            if hit_ship_counter == hit_ship_size:
                for i in range(hit_ship_size):
                    self.robot[hit_ship[i]] = self.SUNK

        # Check victory condition
        self.check_victory()

    #
    # Update the opponent's board with the corresponding position and result
    #
    def update_opponent_board(self, position, result):
        # Assume result is one of the battleship constants 
        self.opponent[pos] = result

        # Check if the whole ship is sunk if it is hit
        if result == self.HIT:

            # Determine which ship that position is in
            ship_loc = None
            for i in self.ship_sizes:
                this_ship = self.opponent_ships[i]
                for j in range(i):
                    if pos == this_ship[j]:
                        ship_loc = i

            # Check if all parts of the ship are hit
            hit_ship = self.opponent_ships[ship_loc]
            hit_ship_size = self.ship_sizes[ship_loc]
            hit_ship_counter = 0

            for i in range(hit_ship_size):
                if self.opponent[hit_ship[i]] == self.HIT:
                    hit_ship_counter += 1

            # If all parts of the ship are hit, change ship to sunk
            if hit_ship_counter == hit_ship_size:
                for i in range(hit_ship_size):
                    self.opponent[hit_ship[i]] = self.SUNK

        # Check victory condition
        self.check_victory()

    #    
    # Check if either side has won, and indicate accordingly.
    #
    def check_victory(self):
        # Check to see the number of sunk ships for both sides.
        robot_sunk_counter = 0
        opponent_sunk_counter = 0

        # Iterate through the board and check the status
        for m in range(self.M):
            for n in range(self.N):
                if self.robot[m,n] == self.HIT or self.robot[m,n] == self.SHIP:
                    return False
                elif self.opponent[m,n] == self.HIT or self.opponent[m,n] == self.SHIP:
                    return False
                elif self.robot[m,n] == self.SUNK:
                    robot_sunk_counter += 1
                elif self.opponent[m,n] == self.SUNK:
                    opponent_sunk_counter += 1
        
        if robot_sunk_counter == sum(self.ship_sizes):
            print('ROBOT WON!')
            #playsound('Robot Win.mp3')
            return True
        elif opponent_sunk_counter == sum(self.ship_sizes):
            print('OPPONENT WON!')
            #playsound('Opponent Win.mp3')
            return True
        else:
            return False

    #    
    # Determine what the position of the board is in xyz space
    #
    def board_to_xyz(self, board):
        #TODO: Implement later
        # Make the function with respect to the aruco market position closest to receiver.
        pass

    #    
    # Determine what a xyz space corresponds to board position
    #
    def xyz_to_board(self, board):
        #TODO: Implement later
        # Make the function with respect to the aruco market position closest to receiver.
        pass

    #    
    # Determine the next target based off the current board space.
    #
    def next_target(self, board, is_cheat):
        #TODO: Implement later
        # Maybe have the cheating thing be updated by a callback function and/or subscriber?
        # First try to do the simulation thing for battl
        pass

    #    
    # Input the ship locations from the visualization node and make the board
    #
    def input_opponent(self, ship_list):
        # Assume ship list is a list of all the ships and their individual locations (list of lists)
        # We may actually want to do the list of inputting here.

        # Check to make sure the ship sizes and the ship list line up
        for i in range(ship_sizes):
            if ship_sizes[i] != len(ship_list[i]):
                raise ValueError('Ship Sizes and Ship Inputs do not match.')
        
        # Fill in the board and the list of ships
        self.opponent_ships = ship_list

        for i in range(len(self.opponent_ships)):
            this_ship = self.opponent_ships[i]
            for j in range(len(this_ship)):
                self.opponent[this_ship[j]] = self.SHIP

#
# Determine the ship board positions (randomly) for the robot
#
def find_ships(board_size, ship_sizes):

    # Eventually return the list of indices corresponding to the ship
    idx = []
    VERTICAL = 0
    HORIZONTAL = 1
    ship_counter = 0

    while ship_counter != len(ship_sizes):
        loc_x = np.floor(np.random.uniform(0, board_size))
        loc_y = np.floor(np.random.uniform(0, board_size))
        orn = np.floor(np.random.uniform(0, 2))  

        this_ship = []
        for i in range(ship_sizes[ship_counter]):
            count_iter = 0
            space_found = False
            while not space_found:
                # Choose the next position for the ship
                if orn == VERTICAL:
                    if (loc_y + i) < board_size:
                        loc_x_next = loc_x
                        loc_y_next = loc_y + i
                    else:
                        loc_x_next = loc_x
                        loc_y_next = (board_size - 1.0) - i 
                elif orn == HORIZONTAL:
                    if (loc_x + i) < board_size:
                        loc_x_next = loc_x + i
                        loc_y_next = loc_y 
                    else:
                        loc_x_next = (board_size - 1.0) - i
                        loc_y_next = loc_y 

                # Test if the space is already occupied
                if (loc_y_next, loc_x_next) not in idx:
                    # Note: x corresponds to cols and y corresponds to rows
                    this_ship.append((loc_y_next, loc_x_next))
                    space_found = True

                count_iter += 1
                # Test if too many iterations occur (then it's impossible to place ship, so need new loc) 
                if count_iter >= 50:
                    print('Impossible to Place Ship!')
                    break

        idx.append(this_ship)
        ship_counter += 1

    print(idx)
    

    #

#
# Create a simulation for battleship and determine the best choice for the next target
#
def simulate_battleship():
    # TODO: Implement later if time. Otherwise, just implement a heuristic
    pass


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    
    ship_sizes = [4, 3, 2]
    board_size = 5
    '''
    ships = np.array([[0, 0, 0, 0, 0], 
        [1, 1, 1, 1, 0],
        [2, 2, 1, 0, 0],
        [3, 3, 0, 0, 1],
        [0, 0, 0, 3, 1]])
    
    ships1 = np.zeros((5,5))

    ships2 = np.array([[0, 0, 0, 0, 0], 
        [4, 4, 4, 4, 0],
        [4, 4, 4, 0, 0],
        [3, 3, 0, 0, 4],
        [0, 0, 0, 3, 4]])

    ships3 = np.array([[0, 0, 0, 0, 0], 
    [4, 4, 4, 4, 0],
    [2, 4, 1, 0, 0],
    [3, 3, 0, 0, 4],
    [0, 0, 0, 3, 2]])

    board0 = Board(ship_sizes, board_size, robot_board=ships)
    board1 = Board(ship_sizes, board_size, robot_board=ships1)
    board2 = Board(ship_sizes, board_size, robot_board=ships2)
    board3 = Board(ship_sizes, board_size, robot_board=ships3)
    '''


    # For testing the random generation of the ships.
    # board = np.zeros(5,5)
    # ship_locations = find_ships(5, [4, 3, 2])
    # for i in range(len(ship_locations)):
    #     board[ship_locations[i]] = 1

    find_ships(board_size, ship_sizes)
