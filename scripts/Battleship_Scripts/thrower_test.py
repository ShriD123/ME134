#!/usr/bin/env python3

import rospy
import math

import sys
sys.path.insert(1, '/home/me134/me134ws/src/ME134/scripts')

import kinematics as kin
import numpy as np
from sensor_msgs.msg   import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot
#from ME134.msg import array
from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5

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
    def __init__(self):
        # Collect the motor names, which defines the dofs (useful to know)
        self.motors = ['Red/4', 'Red/5']
        self.dofs = len(self.motors)
        
        # Create a publisher to send the joint commands. 
        # TODO: When moving to battleship, will want to remove these
        
        #--- FOR RVIZ TESTING ONLY
        #self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        #!-- END RVIZ TESTING
        
        #--- FOR ACTUAL ROBOT
        self.pub = rospy.Publisher("/hebi_thrower/joint_commands", JointState, queue_size=5)
        self.trpub = rospy.Publisher("/tr", String, queue_size=5)
        
        rospy.sleep(0.25)
        
        #--- FOR ACTUAL ROBOT
        # Create subscribers for the general case and events.
        self.sub = rospy.Subscriber('/hebi_thrower/joint_states', JointState, self.callback_actual, queue_size=5)
        
        # Callback event for exit wait condition
        self.sub_exitwait = rospy.Subscriber('/rt', String, self.callback_exitwait)
        
        # # Find the starting positions. 
        msg = rospy.wait_for_message('/hebi_thrower/joint_states', JointState)
        for i in range(self.dofs):
            if (msg.name[i] != self.motors[i]):
                raise ValueError("Motor names don't match")
        self.pos_init = np.array(msg.position).reshape((self.dofs, 1))
        
        #--- FOR RVIZ TESTING ONLY
        #self.pos_init = np.array([np.pi, 0.0]).reshape((2, 1))
        #!-- END RVIZ TESTING

        self.callback_actual(msg)

        # TODO: Create the necessary subscribers for the general case and events.
        
        # self.sub = rospy.Subscriber('/actual', JointState, self.callback_actual)
        # self.sub_e = rospy.Subscriber('/event', String, self.callback_event)
        
        # Grab the robot's URDF from the parameter server.
        # Might have to change this because we'll have multiple URDFs
        #robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        #self.kin = kin.Kinematics(robot, 'world', 'tip')

        # Initialize current time
        self.curr_t = 0.0
        
        # IMPORTANT: GEAR RATIO!
        self.gearratio = 2.0
        
        # Compute trajectory
        # MOVE FROM INITIAL TO LOADING POSITION
        self.LOAD = np.array([-np.pi/2, 0.0]).reshape((2, 1))
        # TRAVEL TIME FOR TRAVERSES
        self.TRAVEL_TIME = 3.0
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

        
        
        # Initialize any helpful global variables
        self.is_waiting = False

        self.msg_sent = False
        
        
    #
    # Update every 10ms!
    #
    def update(self, t):
        # Create an empty joint state message. TODO: Remove when integrating with battleship.py
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
                        trmsg = "Receive"
                        self.trpub.publish(trmsg)
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
        self.curr_pos = cmdmsg.position
        self.curr_vel = cmdmsg.velocity
        self.curr_accel = cmdmsg.effort
        self.curr_t = t


        if (self.curr_pos[1] <= -0.15 or self.curr_pos[1] >= np.pi/2):
            rospy.logerr("Bad theta input")
            rospy.signal_shutdown()

        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)



        

    
    # Callback Function to set the positions based off the actual values
    def callback_actual(self, msg):
        self.curr_pos = msg.position
        self.curr_vel = msg.velocity

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
            self.is_waiting = False
            self.msg_sent = False
            # Populate spline with new trajectory
            self.compute_spline((2, 0))

    #
    # Determine the speed and end position of the thrower for a given target position in xyz space
    #
    def compute_spline(self, target):
        #TODO: To be implemented with a fit to a model.
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
        self.HOLD_TIME = 1.0

        # MOVE TO START POSITION (PRIOR TO LAUNCH)
        self.START = np.array([0.0, 0.0]).reshape((2, 1))
        
        # LAUNCH THE PROJECTILE
        
        # CHANGE THE EXIT VELOCITY HERE!
        self.exit_velocity = 1.92 # Exit velocity in Radians per second (Velocity of HEBI motor)
        self.release_point = np.pi/4.7 # Where projectile is released (Position of HEBI motor)
        
        # Point of release
        self.LAUNCH = np.array([0.0, self.release_point]).reshape((2, 1))
        # Launch time has to be adjusted based on exit velocity. Want the resulting spline to be monotonically increasing
        # Time must be more than Distance / Exit Velocity
        self.LAUNCH_TIME = 2.0*self.release_point / self.exit_velocity
        
        # Velocity at release
        self.launch_vel = np.array([0.0, self.exit_velocity]).reshape((2, 1))
        self.launch_accel = np.array([0.0, 0.0]).reshape((2, 1))
        
        # STOPPING THE ARM AFTER LAUNCH

        self.stop_point = self.release_point*1.3
        self.STOP_TIME = 2*(self.stop_point-self.release_point) / self.exit_velocity
        self.FINAL = np.array([0.0, self.stop_point]).reshape((2, 1))
        self.final_vel = np.array([0.0, 0.0]).reshape((2, 1))
        self.final_accel = np.array([0.0, 0.0]).reshape((2, 1))
        
        # STACK IMPLEMENTATION
        self.trajectory = Trajectory([
            # Go back to loading position
            Goto5(self.curr_t+self.HOLD_TIME+self.LAUNCH_TIME+self.STOP_TIME+2*self.TRAVEL_TIME, 
                self.FINAL, self.LOAD, self.TRAVEL_TIME),
            # Use Quintic spline to stop arm after launch
            QuinticSpline(self.curr_t+self.HOLD_TIME+self.LAUNCH_TIME+2*self.TRAVEL_TIME, 
                self.LAUNCH, self.launch_vel, self.launch_accel, self.FINAL, self.final_vel, self.final_accel, self.STOP_TIME),
            # Use Quintic spline to move from start to release point at specified launch velocity
            QuinticSpline(self.curr_t+self.HOLD_TIME+2*self.TRAVEL_TIME,
                self.START, self.curr_vel, self.curr_accel, self.LAUNCH, self.launch_vel, self.launch_accel, self.LAUNCH_TIME),
            # Travel to launch start position
            Goto5(self.curr_t+self.HOLD_TIME+self.TRAVEL_TIME, 
                self.LOAD, self.START, self.TRAVEL_TIME),
            # Hold at loading position 
            Goto5(self.curr_t+self.TRAVEL_TIME,
                self.LOAD, self.LOAD, self.HOLD_TIME),
            # Move from initial position to loading position. (Initial may be the same as loading position)
            Goto5(self.curr_t,
                self.LOAD, self.LOAD, self.TRAVEL_TIME)])




                            
    
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

    
