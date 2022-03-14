#!/usr/bin/env python3

import rospy
import numpy as np

from detector import Detector
from receiver import Receiver
from thrower import Thrower
from visualize_battle import Visualizer
from algorithm import Board
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
        board_origin = self.detector(soemthing soemtrhing)
        init_state = rospy.wait_for_message('/hebi/joint_states', JointState)


        # Initialize the visualization and board states
        # self.alg = Board()
        self.vis = Visualizer()
        opponent_ships = self.vis.choose_ship_position_rand()
        self.alg = Board()

        # Move the arms to the initial position given their current positions.
        thrower_init_pos = init_state.position[0:2]
        receiver_init_pos = init_state.position[2:7]
        self.receiver = Receiver(receiver_init_pos)
        self.thrower = Thrower(thrower_init_pos)

        # Connect the algorithm and the visualizer 
        # Collect the motor names, which defines the dofs (useful to know)
        # First 2 are Thrower motors, last 5 are receiver motors (with last being for gripper)
        self.motors = ['Red/1', 'Red/5', 'Red/7', 'Red/6', 'Red/4', 'Red/2', 'Red/3']
        self.dofs = len(self.motors)

        # Initialize any helpful global variables
        self.curr_t = 0.0
        self.board_size = 5
        self.ship_sizes = [4, 3, 2]
        self.next_target = (0, 0)
        # Robot board, closer to thrower --> X Values Should be greater than thres
        self.board_thres = 0.0 


        # Create publishers and subscribers for the Detector. 
        self.sack_detect = rospy.Subscriber('/blob_loc', aruco_center, self.callback_blob)

        # self.pub_image = rospy.Publisher('detection_images_thresh', Image, queue_size=10)
        # self.pub_detect = rospy.Publisher('detections', ManyDetections, queue_size=10)

        self.sub_image = rospy.Subscriber('/usb_cam/image_raw', Image, self.detector.aruco_detect)

        # Create publishers and subscribers for the Arms. Note that this combines all working motors.
        self.pub_motors = rospy.Publisher("/hebi/joint_commands", JointState, queue_size=5)

        # Give some time for the publishers and subscribers to connect.
        rospy.sleep(0.50)


    #    
    # Update every 10ms!
    #
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = self.motors

        # Check if both arms are waiting. Then update trajectories.

        # Update both receiver and thrower. All storage of info happens in those update functions.
        q_r, qdot_r, qdotdot_r = self.receiver.update(t)
        q_t, qdot_t, qdotdot_t  = self.thrower.update(t)

        # Send the command (with the current time).
        cmdmsg.position = np.vstack([q_t.reshape((2, 1)), q_r.reshape((5, 1))])
        cmdmsg.velocity = np.vstack([qdot_t.reshape((2, 1)), qdot_r.reshape((5, 1))])
        cmdmsg.effort = np.vstack([qdotdot_t.reshape((2, 1)), qdotdot_r.reshape((5, 1))])
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub_motors.publish(cmdmsg)

        # Save current timestamp 
        self.curr_t = cmdmsg.header.stamp


    #
    # Callback Function for when hackysacks are detected
    #
    def callback_blob(self, msg):
        # Collect the hackysack positions
        hackysacks_x = msg.datax
        hackysacks_y = msg.datay

        # Update the human and robot boards
        for i in range(len(hackysacks_x)):
            loc = (hackysacks_x[i], hackysacks_y[i])
            if loc[0] > self.board_thres:
                # Robot Board
                victory, winner = self.alg.update_robot_board(loc)
            else:
                # Opponent Board
                victory, winner = self.alg.update_opponent_board(loc)
        
        # Update the visualization
        robot_board, robot_ships = self.alg.get_robot_state()
        human_board, human_ships = self.alg.get_opponent_state()

        self.vis.draw_board(robot_board, robot_ships, player='robot')
        self.vis.draw_board(human_board, human_ships, player='human')

        # Check if there is a victory, then showcase it
        if victory:
            # Passes in either 'ROBOT' or 'WINNER'
            self.vis.declare_winner(winner)
        
        # Compute next target
        self.next_target =self.alg.next_target()

###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    # Instantiate the battleship generator object, encapsulating all
    # the computation and local variables.
    battle = Battleship()

    # Prepare a servo loop at 100Hz.
    rate  = 100
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