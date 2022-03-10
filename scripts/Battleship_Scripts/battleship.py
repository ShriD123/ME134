#!/usr/bin/env python3

import rospy
import numpy as np

from detector import Detector
from receiver import Receiver
from thrower import Thrower
from visualize_battle import Visualizer
from algorithm import Bayes
import sys

sys.path.insert(1, '/home/me134/me134ws/src/ME134/scripts')
import kinematics as kin
from sensor_msgs.msg   import JointState

from sensor_msgs.msg import Image
from detector_demo.msg import SingleDetection, ManyDetections


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

    print(idx)



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
        self.detector = Detector(h_lims=(100, 200), s_lims=(100, 230), v_lims=(100, 250))

        init_state = rospy.wait_for_message('/hebi/joint_states', JointState)

        thrower_init_pos = init_state.position[0:2]
        thrower_init_vel = init_state.velocity[0:2]

        receiver_init_pos = init_state.position[2:7]
        receiver_init_vel = init_state.velocity[2:7]

        self.receiver = Receiver(receiver_init_pos)
        self.thrower = Thrower(thrower_init_pos)
        self.vis = Visualizer()
        self.bayes = Bayes()

        # Collect the motor names, which defines the dofs (useful to know)
        # First 2 are Thrower motors, last 5 are receiver motors (with last being for gripper)
        # TODO: Change the order of the motors names/wiring to match this scheme
        self.motors = ['Red/4', 'Red/5', 'Red/7', 'Red/6', 'Red/1', 'Red/2', 'Red/3']
        self.dofs = len(self.motors)

        # Initialize any helpful global variables
        self.curr_t = 0.0
        self.board_size = 5
        self.ship_sizes = [4, 3, 2]
        '''
        # Initialize the board, saved as a 5x5 numpy array
        # 1 means a ship is at that location, while 0 means it is empty.
        self.board = np.zeros(self.board_size, self.board_size)
        ship_locations = self.find_ships(self.board_size, self.ship_sizes)
        for i in range(len(ship_locations)):
            self.board[ship_locations[i]] = 1

        # Visualize the board. 
        #TODO: Figure out how to differentiate between player and robot and different ships
        self.vis.draw_board(self.board)
        '''
        # Create publishers and subscribers for the Detector. 
        # Might not need these 2 publishers
        self.pub_image = rospy.Publisher('detection_images_thresh', Image, queue_size=10)
        self.pub_detect = rospy.Publisher('detections', ManyDetections, queue_size=10)

        self.sub_image = rospy.Subscriber('/usb_cam/image_raw', Image, self.detector.aruco_detect)
        #self.sub_blob = rospy.Subscriber('/blob_loc', aruco_center, self.callback_detect)

        # Create publishers and subscribers for the Arms. Note that this combines all working motors.
        #self.pub_motors = rospy.Publisher("/joint_states", JointState, queue_size=5)
        self.pub_motors = rospy.Publisher("/hebi/joint_commands", JointState, queue_size=5)
        self.sub_motors = rospy.Subscriber('/hebi/joint_states', JointState, self.callback, queue_size=5)

        # Give some time for the publishers and subscribers to connect.
        rospy.sleep(0.50)

        #TODO: Move all the arms to a starting position.
        #self.THROW_START = np.array([0.0, 

        #TODO: Let player choose their board (how to implement?)
#])
        #TODO: Make a noise to indicate when ready (let player take first turn?)


    #    
    # Update every 10ms!
    #
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = self.motors

        # Save current timestamp for splines
        self.curr_t = t

        # Check if any detection has occurred
        #if 
        #msg = self.detector.is_detect()             # Need to implement is_detect()
        # TODO: Implement what to do with the corresponding detection later.

        # Check if both arms are waiting. Then update trajectories.

        # Update both receiver and thrower. All storage of info happens in those update functions.
        q_r, qdot_r, qdotdot_r = self.receiver.update(t)
        q_t, qdot_t, qdotdot_t  = self.thrower.update(t)
        #qdotdot_r = self.receiver.gravity()              # Definitely wrong, but will want something like this
        #qdotdot_t = self.thrower.gravity()              # Definitely wrong, but will want something like this

        # Send the command (with the current time).
        cmdmsg.position = np.vstack([q_t.reshape((2, 1)), q_r.reshape((5, 1))])
        cmdmsg.velocity = np.vstack([qdot_t.reshape((2, 1)), qdot_r.reshape((5, 1))])
        cmdmsg.effort = np.vstack([qdotdot_t.reshape((2, 1)), qdotdot_r.reshape((5, 1))])
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub_motors.publish(cmdmsg)

    # def callback_detect(self,msg):
    #     if msg.blob == "No Blob":
    #         pass
    #     else:
    #         pass

            # Update Thrower spline ???

        
    #
    # Callback Function 
    #
    def callback(self, msg):
        #TODO: Figure out number of callback functions and how to implement
        #TODO: As is currently, will want to call the update/gravity functions of both arms
        pass
                            
    

###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    #rospy.init_node('Battle')

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

    # board = np.zeros(5,5)
    # ship_locations = find_ships(5, [4, 3, 2])
    # for i in range(len(ship_locations)):
    #     board[ship_locations[i]] = 1


