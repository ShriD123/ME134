#!/usr/bin/env python3

import rospy
import math
import time

import sys
sys.path.insert(1, '/home/me134/me134ws/src/ME134/scripts')

import kinematics as kin
import numpy as np
from sensor_msgs.msg   import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot
#from ME134.msg import array
from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
from ME134.msg import aruco_center
from playsound import playsound

'''VERSION OF THROWER CODE INTENDED ONLY FOR TESTING THROWER WITH VELOCITY.'''


###############################################################################
#
#  Trajectory Class
#
class Trajectory:
    #
    # Initialize
    #
    def __init__(self, list_splines=[]):
        self.splines = list_splines
        if len(list_splines) == 0:
            self.curr_spline = None
        else:
            self.curr_spline = self.splines.pop()

    #
    # Returns the space of the current trajectory
    #    
    def traj_space(self):
        return self.curr_spline.space()
    
    #
    # Returns the trajectory at the top of the stack
    #
    def pop_spline(self):
        self.curr_spline = self.splines.pop()

    #
    # Adds a trajectory to the stack (LIFO)
    #
    def add_spline(self, next_spline):
        self.splines.append(next_spline)

    #
    # Checks if there are no trajectories
    #
    def is_empty(self):
        if len(self.splines) == 0:
            return True
        else:
            return False

    #
    # Check the duration of the current trajectory.
    #
    def duration(self):
        return self.curr_spline.duration()
    
    #
    # Check the starting time of the current trajectory.
    #
    def start_time(self):
        return self.curr_spline.start_time()
    
    #
    # Determine the pos and vel from current trajectory
    #
    def update(self, t):
        (pos, vel) = self.curr_spline.evaluate(t)
        return (pos, vel)


###############################################################################
#
#  Thrower Class... For now, it will be implemented as a standalone class (soon to be integrated into Battleship)
#
class Thrower:
    #
    # Initialize.
    #
    def __init__(self, init_pos):
        # Collect the motor names, which defines the dofs (useful to know)
        self.motors = ['Red/1', 'Red/5']
        self.dofs = len(self.motors)
        
        # Create a publisher to send the joint commands. 
        
        #--- FOR RVIZ TESTING ONLY
        #self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        #!-- END RVIZ TESTING
        
        #--- FOR ACTUAL ROBOT
        #self.pub = rospy.Publisher("/hebi_thrower/joint_commands", JointState, queue_size=5)
        self.trpub = rospy.Publisher("/tr", String, queue_size=5)
        
        rospy.sleep(0.25)
        
        #--- FOR ACTUAL ROBOT
        # Create subscribers for the general case and events.
        #self.sub = rospy.Subscriber('/hebi_thrower/joint_states', JointState, self.callback_actual, queue_size=5)
        
        # Callback event for exit wait condition
        self.sub_exitwait = rospy.Subscriber('/rt', String, self.callback_exitwait)
        
        # # Find the starting positions. 
        # msg = rospy.wait_for_message('/hebi_thrower/joint_states', JointState)
        # for i in range(self.dofs):
        #     if (msg.name[i] != self.motors[i]):
        #         raise ValueError("Motor names don't match")
        self.pos_init = np.array(init_pos).reshape((self.dofs, 1))
        
        #--- FOR RVIZ TESTING ONLY
        #self.pos_init = np.array([np.pi, 0.0]).reshape((2, 1))
        #!-- END RVIZ TESTING

        self.curr_pos = init_pos
        self.curr_vel = np.array([0.0,0.0]).reshape((2,1))

        # Initialize current time
        self.curr_t = 0.0
        
        # IMPORTANT: GEAR RATIO!
        self.gearratio = 2.0
        
        # Compute trajectory
        # MOVE FROM INITIAL TO LOADING POSITION
        self.LOAD = np.array([-np.pi/2, 0.0]).reshape((2, 1))
        # TRAVEL TIME FOR TRAVERSES
        self.TRAVEL_TIME = 3.0
        self.INIT_TIME = 2.0
        self.trajectory = Trajectory([Goto5(self.curr_t, self.pos_init, self.LOAD, self.TRAVEL_TIME)])


        # Initialize the gravity parameters

        # For a 3.33 gear ratio, A = 0, B = 3.27, B_hackysack = 1.04
        # For a 2:1 gear ratio, theoretically B = 1.96, B_hackysack = 0.624
        self.grav_A = 0.0
        self.grav_B = 1.96

        # Addition to gravity term due to hackysack
        self.grav_B_hackysack = 0.624

        # If we want to float the arm
        self.float = False

        # self.speed = np.array([[2.098, 2.065, 2.05,  2.058, 2.091],\
        #                 [2.194, 2.181, 2.162, 2.164, 2.191],\
        #                 [2.323, 2.307, 2.279, 2.280, 2.297],\
        #                 [2.436, 2.412, 2.387, 2.402, 2.422],\
        #                 [2.571, 2.545, 2.524, 2.528, 2.545]])

        # self.angle = np.array([[0.23, 0.14, 0.04, -0.09, -0.19],\
        #                        [0.23, 0.13, 0.04, -0.08, -0.16],\
        #                        [0.21, 0.14, 0.04, -0.05, -0.14],\
        #                        [0.23, 0.13, 0.04, -0.05, -0.12],\
        #                        [0.2,  0.13, 0.06, -0.02, -0.1]])

        speed = np.array([2.091, 2.058, 2.05, 2.065, 2.098,
                               2.191, 2.164, 2.162, 2.181, 2.194,
                               2.297, 2.280, 2.279, 2.307, 2.323,
                               2.422, 2.402, 2.387, 2.412, 2.436,
                               2.545, 2.528, 2.524, 2.545, 2.571])
        self.speed = speed.reshape((5, 5))
        
        angle = np.array([[-0.19, -0.09, 0.04, 0.14, 0.23],
                               [-0.16, -0.08, 0.04, 0.13, 0.23],
                               [-0.14, -0.05, 0.04, 0.14, 0.21],
                               [-0.12, -0.05, 0.04, 0.13, 0.23],
                               [-0.1, -0.02, 0.06, 0.13, 0.2]])
        self.angle = angle.reshape((5, 5))

        
        
        # Initialize any helpful global variables
        self.is_waiting = False

        self.msg_sent = False
        self.msg_length = 0

        self.target = (0, 0)
        self.board_size = 5
        
        
    #
    # Update every 10ms!
    #
    def update(self, t, target):
        # Update the next target position
        self.target = target

        # Create an empty joint state message. 
        cmdmsg = JointState()
        
        #--- FOR RVIZ TESTING ONLY
        #cmdmsg.name = ['Pan', 'Tilt']
        #!-- END RVIZ TESTING
        
        cmdmsg.name = self.motors

        # CODE FOR TESTING FLOAT
        if self.float:
            # Set q_des = q_actual to make the arm "float"
            nan = float('nan')
            cmdmsg.position = np.array([nan, nan]).reshape((2,1))
            cmdmsg.velocity = np.array([nan, nan]).reshape((2,1))
            cmdmsg.effort = self.gravity(self.curr_pos, hackysack=True)
        
        else:
            # If the current segment is done, shift to the next.
            if (t-self.trajectory.start_time()) >= self.trajectory.duration():
                if self.trajectory.is_empty():
                    self.is_waiting = True
                    if not self.msg_sent:
                        print("waiting start")
                        # detect_throw = False
                        # first_detection = rospy.wait_for_message('/blob_loc', aruco_center)
                        # start_msg_length = len(first_detection.datax)
                        # # Tell the opponent to throw
                        playsound('/home/me134/me134ws/src/ME134/sounds/go.mp3', False)
                        print("Played sound")
                        # while not detect_throw:
                        #     detection = rospy.wait_for_message('/blob_loc', aruco_center)
                        #     print("Check len change")
                        #     if len(detection.datax) > start_msg_length:
                        #         print("len changed")
                        #         print(len(detection.datax))
                        #         detect_throw = True
                        # print("send receive")
                        # trmsg = "Receive"
                        # self.trpub.publish(trmsg)
                        self.msg_sent = True
                else:
                    self.trajectory.pop_spline()

            if self.is_waiting:
                # Make the arm float in the loading position if just waiting.
                cmdmsg.position = self.LOAD
                cmdmsg.velocity = np.array([0.0, 0.0]).reshape((2, 1))
                cmdmsg.effort = self.gravity(self.curr_pos)
            else:
                (cmdmsg.position, cmdmsg.velocity) = self.trajectory.update(t)
                cmdmsg.effort = self.gravity(self.curr_pos, hackysack=True)


        # Store the command message
        self.curr_pos = np.array(cmdmsg.position).reshape((2, 1))
        self.curr_vel = np.array(cmdmsg.velocity).reshape((2, 1))
        self.curr_accel = np.array(cmdmsg.effort).reshape((2, 1))
        self.curr_t = t


        if (self.curr_pos[1] <= -0.15 or self.curr_pos[1] >= np.pi/2):
            rospy.logerr("Bad theta input")
            rospy.logerr(self.curr_pos[1])
            rospy.signal_shutdown()

        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        #self.pub.publish(cmdmsg)
        return self.curr_pos, self.curr_vel, self.curr_accel


    # Callback Function to set the positions based off the actual values
    def callback_actual(self, msg):
        self.curr_pos = np.array(msg.position).reshape((2, 1))
        self.curr_vel = np.array(msg.velocity).reshape((2, 1))

    #
    # Gravity Compensation Function
    #
    def gravity(self, pos, hackysack=False):
        # If a hackysack is loaded, add adjustment value to gravity parameter
        if hackysack:
            grav_B_term = self.grav_B + self.grav_B_hackysack
        # Otherwise use as is
        else:
            grav_B_term = self.grav_B

        theta_1 = pos[1] * self.gearratio
        tau = self.grav_A * math.sin(theta_1) + grav_B_term * math.cos(theta_1)
        return np.array([0, tau]).reshape((self.dofs,1)) 

    # Callback function to exit waiting condition and go into trajectory
    def callback_exitwait(self, msg):
        rospy.loginfo('Hello! I heard %s', msg.data)
        # Exit wait if wait is true
        if self.is_waiting:
            # Populate spline with new trajectory
            time.sleep(1)
            #print('Done wait')
            curr_state  = rospy.wait_for_message('/hebi/joint_states', JointState)
            current_effort = curr_state.effort
            sack_holder_array = rospy.wait_for_message('/blob_loc', aruco_center)
            for ind,locx in enumerate(sack_holder_array.datax):
                locy = sack_holder_array.datay[ind]
                if (locx < 0.61 and locx > 0.54 and locy > -0.19 and locy < -0.13):
                    if current_effort[1] > 2.15:
                        rospy.logerr('Hackysack in Thrower')
                        self.is_waiting = False
                        self.msg_sent = False
                        self.compute_spline()
                        sack_in_thrower = True
                        break
                else:
                    sack_in_thrower = False
            
            if not sack_in_thrower:
                rospy.logerr('No Hackysack in Thrower')
                trmsg = "Receive"
                self.trpub.publish(trmsg)
                self.msg_sent = True

    #
    # Determine the speed and end position of the thrower for a given target position in xyz space
    #
    def compute_spline(self):
        # For now, we will just use kinematic equations (once testing of arm is complete)

        # Initialize the state of the robot
        self.curr_pos = self.LOAD
        self.curr_vel = np.array([0.0, 0.0]).reshape((2, 1))
        self.curr_accel = np.array([0.0, 0.0]).reshape((2, 1))

        # MOVE FROM INITIAL TO LOADING POSITION
        self.LOAD = np.array([-np.pi/2, 0.0]).reshape((2, 1))
        # TRAVEL TIME FOR TRAVERSES
        self.TRAVEL_TIME = 3.0
        # HOLD
        self.HOLD_TIME = 0.1

        # MOVE TO START POSITION (PRIOR TO LAUNCH)
        self.START = np.array([self.angle[self.target], 0.0]).reshape((2, 1))
        
        # LAUNCH THE PROJECTILE
        
        # CHANGE THE EXIT VELOCITY HERE!
        self.exit_velocity = self.speed[self.target] # Exit velocity in Radians per second (Velocity of HEBI motor)
        self.release_point = np.pi/10 # Where projectile is released (Position of HEBI motor)
        
        # Point of release
        self.LAUNCH = np.array([self.angle[self.target], self.release_point]).reshape((2, 1))
        # Launch time has to be adjusted based on exit velocity. Want the resulting spline to be monotonically increasing
        # Time must be more than Distance / Exit Velocity
        self.LAUNCH_TIME = 2.0*self.release_point / self.exit_velocity
        
        # Velocity at release
        self.launch_vel = np.array([0.0, self.exit_velocity]).reshape((2, 1))
        self.launch_accel = np.array([0.0, 0.0]).reshape((2, 1))
        
        # STOPPING THE ARM AFTER LAUNCH

        self.stop_point = self.release_point*1.3
        self.STOP_TIME = 2*(self.stop_point-self.release_point) / self.exit_velocity
        self.FINAL = np.array([self.angle[self.target], self.stop_point]).reshape((2, 1))
        self.final_vel = np.array([0.0, 0.0]).reshape((2, 1))
        self.final_accel = np.array([0.0, 0.0]).reshape((2, 1))

        # STACK IMPLEMENTATION
        self.trajectory = Trajectory([
            # Go back to loading position
            Goto5(self.curr_t+self.HOLD_TIME+self.LAUNCH_TIME+self.STOP_TIME+self.TRAVEL_TIME+self.INIT_TIME, 
                self.FINAL, self.LOAD, self.TRAVEL_TIME),
            # Use Quintic spline to stop arm after launch
            QuinticSpline(self.curr_t+self.HOLD_TIME+self.LAUNCH_TIME+self.TRAVEL_TIME+self.INIT_TIME, 
                self.LAUNCH, self.launch_vel, self.launch_accel, self.FINAL, self.final_vel, self.final_accel, self.STOP_TIME),
            # Use Quintic spline to move from start to release point at specified launch velocity
            QuinticSpline(self.curr_t+self.HOLD_TIME+self.TRAVEL_TIME+self.INIT_TIME,
                self.START, self.curr_vel, self.curr_accel, self.LAUNCH, self.launch_vel, self.launch_accel, self.LAUNCH_TIME),
            # Travel to launch start position
            Goto5(self.curr_t+self.HOLD_TIME+self.INIT_TIME, 
                self.LOAD, self.START, self.TRAVEL_TIME),
            # Hold at loading position 
            Goto5(self.curr_t+self.INIT_TIME,
                self.LOAD, self.LOAD, self.HOLD_TIME),
            # Move from initial position to loading position. (Initial may be the same as loading position)
            Goto5(self.curr_t,
                self.LOAD, self.LOAD, self.INIT_TIME)])
        # self.INIT_TIME = 2.0


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('Thrower')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    thrower = Thrower()

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
        thrower.update(t)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()

    
