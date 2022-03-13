#!/usr/bin/env python3

import rospy
import math
import time 
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
    def __init__(self, opponent_ships, aruco_position=None, robot_board=None, opponent_board=None):
        # Initialize any important variables
        self.M = 5
        self.N = 5
        self.ship_sizes = [4, 3, 2]

        self.board_origin = aruco_position      # FIGURE OUT LATER HOW EXACTLY TO INPUT THIS... important for board to xyz and xyz to board fns
        self.is_cheat = False                   # Make a publisher and subscriber to update this on the fly

        # Constants to discern between different states
        self.WATER = 0
        self.SHIP = 1
        self.HIT = 2
        self.MISS = 3
        self.SUNK = 4

        # Initialize the 5X5 boards and ship list for the robot and the opponent
        self.robot_ships = find_ships(board_size, self.ship_sizes)
        self.opponent_ships = opponent_ships

        self.robot = np.zeros((self.M, self.N))
        self.input_robot(self.robot_ships)
        self.opponent = np.zeros((self.M, self.N))  
        self.input_opponent(self.opponent_ships)

        # If we ever need to manually adjust the boards
        if robot_board is not None:
            self.robot = robot_board
        
        if opponent_board is not None:
            self.opponent = opponent_board

    #    
    # Update the robot's board with the corresponding hackysack position
    #
    def update_robot_board(self, pos):
        board_pos, result = self.check_robot_result(pos)

        self.robot[board_pos] = result

        # Check if the whole ship is sunk if it is hit
        if result == self.HIT:

            # Determine which ship that position is in
            ship_loc = None
            for i in self.ship_sizes:
                this_ship = self.robot_ships[i]
                for j in range(i):
                    if board_pos == this_ship[j]:
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
        return self.check_victory()

    #
    # Update the opponent's board with the corresponding hackysack position
    #
    def update_opponent_board(self, pos):
        board_pos, result = self.check_opponent_result(pos)

        self.opponent[board_pos] = result

        # Check if the whole ship is sunk if it is hit
        if result == self.HIT:

            # Determine which ship that position is in
            ship_loc = None
            for i in self.ship_sizes:
                this_ship = self.opponent_ships[i]
                for j in range(i):
                    if board_pos == this_ship[j]:
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
        return self.check_victory()


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
            return True, 'ROBOT'
        elif opponent_sunk_counter == sum(self.ship_sizes):
            print('OPPONENT WON!')
            #playsound('Opponent Win.mp3')
            return True, 'OPPONENT'
        else:
            return False, None

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
    def next_target(self, is_cheat):
        # TODO: Input the list of indices in terms of probability order later.
        prob_map = []
        target_pos = 0          # Return this as an index for now... see what the thrower compute spline does

        # Use to determine if should use opponent's board or not
        if self.is_cheat:
            data_threshold = 0.80
        else:
            data_threshold = 0.20

        rand_val = np.random.uniform()

        if rand_val < data_threshold:
            # Find a ship position not already hit in opponent's grid
            for i in range(self.M):
                for j in range(self.N):
                    if self.opponent[i,j] == self.SHIP:
                        return (i,j)
        else:
            # Use the next value in the probability map, if there is a ship and not hit
            for k in enumerate(prob_map):
                if self.opponent[k] == self.SHIP:
                    return k

        # If no ships remaining, we should be victorious, so value error
        print(self.robot)
        print(self.opponent)
        raise ValueError('No Ships Remaining, Should be Victorious')


    #    
    # Input the robot ship locations during initialization and make the board
    #
    def input_robot(self, ship_list):
        # Assume ship list is a list of all the ships and their individual locations (list of lists)

        # Check to make sure the ship sizes and the ship list line up
        for i in range(len(self.ship_sizes)):
            if self.ship_sizes[i] != len(ship_list[i]):
                raise ValueError('Ship Sizes and Ship Inputs do not match.')

        for i in range(len(ship_list)):
            this_ship = ship_list[i]
            for j in range(len(this_ship)):
                self.robot[this_ship[j]] = self.SHIP

    #    
    # Input the opponent ship locations during initialization and make the board
    #
    def input_opponent(self, ship_list):
        # Assume ship list is a list of all the ships and their individual locations (list of lists)

        # Check to make sure the ship sizes and the ship list line up
        for i in range(len(self.ship_sizes)):
            if self.ship_sizes[i] != len(ship_list[i]):
                raise ValueError('Ship Sizes and Ship Inputs do not match.')

        for i in range(len(ship_list)):
            this_ship = ship_list[i]
            for j in range(len(this_ship)):
                self.opponent[this_ship[j]] = self.SHIP

    # 
    # Check the hackysack position on robot board and see what result in the game that gives you
    #
    def check_robot_result(self, hackysack_pos):
        # Find the board position pertaining to the hackysack
        board_pos = xyz_to_board(hackysack_pos)         # Should return a tuple of indices on the board

        if self.robot[board_pos] == self.WATER:
            return board_pos, self.MISS
        elif self.robot[board_pos] == self.SHIP:
            return board_pos, self.HIT
        else:
            # Position already been hit, so just return that again
            return board_pos, self.robot[board_pos]
        
    # 
    # Check the hackysack position on opponent board and see what result in the game that gives you
    #
    def check_opponent_result(self, hackysack_pos):
        # Find the board position pertaining to the hackysack
        board_pos = xyz_to_board(hackysack_pos)         # Should return a tuple of indices on the board

        if self.opponent[board_pos] == self.WATER:
            return board_pos, self.MISS
        elif self.opponent[board_pos] == self.SHIP:
            return board_pos, self.HIT
        else:
            # Position already been hit, so just return that again
            return board_pos, self.opponent[board_pos]

    # 
    # Get the robot board and ships (for visualization purposes)
    # 
    def get_robot_state(self):
        return self.robot, self.robot_ships
    
    # 
    # Get the opponent board and ships (for visualization purposes)
    # 
    def get_opponent_state(self):
        return self.opponent, self.opponent_ships
############################################################################################
#
#   HELPER FUNCTIONS
#

#
# Determine the ship board positions (randomly) for the robot
#
def find_ships(board_size, ship_sizes):
    # Eventually return the list of indices corresponding to the ship
    idx = []
    VERTICAL = 0
    HORIZONTAL = 1

    for i, e in enumerate(ship_sizes):
        valid_ship = []
        ship_not_found = True

        while ship_not_found:
            # Setup the random ship parameters
            this_ship = []
            overlap = False
            loc_x = int(np.floor(np.random.uniform(0, board_size)))
            loc_y = int(np.floor(np.random.uniform(0, board_size)))
            orn = int(np.floor(np.random.uniform(0, 2)))

            # Find the possible ship positions from the current location
            for k in range(e):
                if orn == VERTICAL:
                    if (loc_y + k) < board_size:
                        loc_x_next = loc_x
                        loc_y_next = loc_y + k
                    else:
                        loc_x_next = loc_x
                        loc_y_next = (board_size - 1) - k 
                elif orn == HORIZONTAL:
                    if (loc_x + k) < board_size:
                        loc_x_next = loc_x + k
                        loc_y_next = loc_y 
                    else:
                        loc_x_next = (board_size - 1) - k
                        loc_y_next = loc_y 
                # Note: y corresponds to rows, x corresponds to columns
                this_ship.append((loc_y_next, loc_x_next))
            
            # See if the ship overlaps with any ships already made
            for x in range(len(idx)):
                for y in this_ship:
                    if y in idx[x]:
                        overlap = True
            
            # Add the ship to the list if there's no overlap
            if not overlap:
                ship_not_found = False
                valid_ship = this_ship

        idx.append(valid_ship)  
    
    return idx
            
#
# Create a simulation for battleship and determine the best choice for the next target
#
def simulate_battleship(board_size, ship_sizes):
    N = int(2.5e7)
    start_time = time.time()
    board_sum = np.zeros((board_size, board_size))
    for i in range(N):
        # Create the random board for this iteration
        list_ships = find_ships(board_size, ship_sizes)

        # Sum the locations of that ship.
        for j in range(len(list_ships)):
            for k, val in enumerate(list_ships[j]):
                board_sum[val] += 1

        print('Iterations Complete: ', i+1, end='\r')
    board_prob = board_sum / N
    print()
    print(board_sum)
    print(board_prob)

    time_taken = time.time() - start_time
    print('Time Taken: ',time_taken)


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    
    ship_sizes = [4, 3, 2]
    board_size = 5
    simulate_battleship(board_size, ship_sizes)

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