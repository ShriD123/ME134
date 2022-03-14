#!/usr/bin/env python3

from pyexpat.model import XML_CTYPE_MIXED
import rospy
import math
import time 
import numpy as np
from std_msgs.msg import String
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
    def __init__(self, opponent_ships, aruco_position, board_thres, robot_board=None, opponent_board=None):
        # Initialize any important variables
        self.M = 5
        self.N = 5
        self.ship_sizes = [4, 3, 2]

        self.board_origin = aruco_position
        self.is_cheat = False              
        self.thres = board_thres     

        # Make a publisher and subscriber to update the cheating function
        self.pub_cheat = rospy.Publisher('/difficulty', String, queue_size=10)
        self.sub_cheat = rospy.Subscriber('/difficulty', String, self.callback_alg)

        # Constants to discern between different states
        self.WATER = 0
        self.SHIP = 1
        self.HIT = 2
        self.MISS = 3
        self.SUNK = 4

        # Initialize the 5X5 boards and ship list for the robot and the opponent
        self.robot_ships = find_ships(self.M, self.ship_sizes)
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

        if board_pos is None or result is None:
            return False, None

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

        if board_pos is None or result is None:
            return False, None

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
    def xyz_to_board(self, sack_pos):
        # Return None if it is not in any of the boards (UPDATE OTHER FUNCTIONS FOR THIS)
        x_idx = None
        y_idx = None
        rel_pos = [0, 0]

        bounds = [0.0, 0.085, 0.175, 0.265, 0.355, 0.440]

        p0 = (0.56, 0.5)        # Vector to (0,0) for robot board
        p1 = (-0.05, 0.5)       # Vector to (0,0) for opponent board

        # Determine which side of the board the sack is on and its relative position
        if sack_pos[0] > self.thres:
            # Robot Board
            rel_pos[0] = sack_pos[0] - (p0[0] + self.board_origin[0])
            rel_pos[1] = sack_pos[1] - (p0[1] + self.board_origin[1])
        else:
            # Opponent Board
            rel_pos[0] = sack_pos[0] - (p1[0] + self.board_origin[0])
            rel_pos[1] = sack_pos[1] - (p1[1] + self.board_origin[1])
        
        # Determine the x index using the bounds (which are symmetric)
        if rel_pos[0] > bounds[0] and rel_pos[0] <= bounds[1]:
            x_idx = 0
        elif rel_pos[0] > bounds[1] and rel_pos[0] <= bounds[2]:
            x_idx = 1
        elif rel_pos[0] > bounds[2] and rel_pos[0] <= bounds[3]:
            x_idx = 2
        elif rel_pos[0] > bounds[3] and rel_pos[0] <= bounds[4]:
            x_idx = 3
        elif rel_pos[0] > bounds[4] and rel_pos[0] <= bounds[5]:
            x_idx = 4
        
        # Determine the y index using the bounds (which are symmetric)
        if rel_pos[1] > bounds[0] and rel_pos[1] <= bounds[1]:
            y_idx = 0
        elif rel_pos[1] > bounds[1] and rel_pos[1] <= bounds[2]:
            y_idx = 1
        elif rel_pos[1] > bounds[2] and rel_pos[1] <= bounds[3]:
            y_idx = 2
        elif rel_pos[1] > bounds[3] and rel_pos[1] <= bounds[4]:
            y_idx = 3
        elif rel_pos[1] > bounds[4] and rel_pos[1] <= bounds[5]:
            y_idx = 4

        return (x_idx, y_idx)

    #    
    # Determine the next target based off the current board space.
    #
    def next_target(self):
        prob_map = [(3,3), (2,3), (3,2), (2,2), (4,3), (3,4), (4,2), (2,4), (3,1),
                    (1,3), (2,1), (1,2), (4,4), (1,1), (4,1), (1,4), (0,3), (3,0),
                    (2,0), (0,2), (4,0), (0,4), (0,1), (1,0), (0,0)]

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
        board_pos = self.xyz_to_board(hackysack_pos)         # Should return a tuple of indices on the board

        # If the hackysack has not hit any spot on the board, return None
        if board_pos[0] is None or board_pos[1] is None:
            return None, None

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
        board_pos = self.xyz_to_board(hackysack_pos)         # Should return a tuple of indices on the board

                # If the hackysack has not hit any spot on the board, return None
        if board_pos[0] is None or board_pos[1] is None:
            return None, None

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

    #
    # Callback function to change if cheating is occurring or not
    #
    def callback_alg(self, msg):
        if self.is_cheat:
            self.is_cheat = False
        else:
            self.is_cheat = True


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