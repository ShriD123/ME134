#!/usr/bin/env python3

import rospy
import math

import sys
# Get path of parent directory for kinematics and splines import
sys.path.insert(1, '/home/me134/me134ws/src/ME134/scripts')

import kinematics as kin
import numpy as np
from sensor_msgs.msg   import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot
from ME134.msg import array
from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5

'''This code encapsulates the functionality for the thrower arm as well as its
corresponding error sensing and the necessary subscribers for itself.'''


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
        #self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        self.pub = rospy.Publisher("/hebi/joint_commands", JointState, queue_size=5)
        rospy.sleep(0.25)

        # Create subscribers for the general case and events.
        self.sub = rospy.Subscriber('/hebi/joint_states', JointState, self.callback_actual, queue_size=5)
        
        
        # # Find the starting positions. 
        msg = rospy.wait_for_message('/hebi/joint_states', JointState)
        for i in range(self.dofs):
            if (msg.name[i] != self.motors[i]):
                raise ValueError("Motor names don't match")
        self.pos_init = np.array(msg.position).reshape((self.dofs, 1))
        #self.pos_init = np.array([0.0, 0.0]).reshape((2, 1))

        self.callback_actual(msg)

        # TODO: Create the necessary subscribers for the general case and events.
        # self.sub = rospy.Subscriber('/actual', JointState, self.callback_actual)
        # self.sub_e = rospy.Subscriber('/event', String, self.callback_event)
        self.sub_hackysack = rospy.Subscriber('/force_sensor', JointState, self.callback_hackysack)
        
        # Grab the robot's URDF from the parameter server.
        # Might have to change this because we'll have multiple URDFs
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')

        # Initialize the state of the robot
        self.curr_pos = self.pos_init
        self.curr_vel = np.array([0.0, 0.0]).reshape((2, 1))
        self.curr_t = 0.0
        self.curr_accel = np.array([0.0, 0.0]).reshape((2, 1))

        # Initialize the trajectory
        self.START = np.array([np.pi/2, 0.0]).reshape((2, 1))
        self.LAUNCH = np.array([0.0, 0.0]).reshape((2, 1))
        self.TRAJ_TIME = 3.0
        self.trajectory = Trajectory([Goto5(self.curr_t, self.pos_init, self.START, self.TRAJ_TIME)])

        # Initialize the gravity parameters TODO: Tune and test these parameters for our 2DOF

        # For a 3.33 gear ratio, A = 0, B = 3.27, B_hackysack = 1.04
        # For a 2.5 gear ratio, 
        self.grav_A = 0.0
        self.grav_B = 3.27

        # Addition to gravity term due to hackysack
        self.grav_B_hackysack = 1.04

        # If we want to float the arm
        self.float = True

        # GEAR RATIO!
        self.gearratio = 3.33
        
        # Initialize any helpful global variables
        self.is_waiting = False
    #
    # Update every 10ms!
    #
    def update(self, t):
        # Create an empty joint state message. TODO: Remove when integrating with battleship.py
        cmdmsg = JointState()
        cmdmsg.name = self.motors

        # # TODO: Code for tuning gravity... Delete when completed tuning.
        # # Set q_des = q_actual to make the arm "float"
        # nan = float('nan')
        # cmdmsg.position = np.array([nan, nan, nan]).reshape((3,1))
        # cmdmsg.velocity = np.array([nan, nan, nan]).reshape((3,1))
        # cmdmsg.effort = self.gravity(self.curr_pos)

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
                else:
                    self.trajectory.pop_spline()

            if self.is_waiting:
                # Make the arm float in the starting position if just waiting.
                cmdmsg.position = self.START
                cmdmsg.velocity = np.array([0.0, 0.0]).reshape((2, 1))
                cmdmsg.effort = self.gravity(self.curr_pos)
            else:
                # Update with respect to the current trajectory.
                if (self.trajectory.traj_space() == 'Joint'):
                    (cmdmsg.position, cmdmsg.velocity) = self.trajectory.update(t)
                    
                elif (self.trajectory.traj_space() == 'Task'):
                    # TODO: Update for a 2DOF Arm... Admittedly, we could just bypass this by never needing this.
                    # TODO: Consider if we even need this set of code... Right now it will fail, but do we need task space?
                    (x, xdot) = self.trajectory.update(t)
                    cart_pos = np.array(x).reshape((self.dofs,1))
                    cart_vel = np.array(xdot).reshape((self.dofs,1))
                    
                    cmdmsg.position = self.kin.ikin(cart_pos, self.curr_pos)
                    (T, J) = self.kin.fkin(cmdmsg.position)
                    cmdmsg.velocity = np.linalg.inv(J[0:3,0:3]) @ cart_vel
                else:
                    raise ValueError('Unknown Spline Type')

                cmdmsg.effort = self.gravity(self.curr_pos)

        # Store the command message
        self.curr_pos = cmdmsg.position
        self.curr_vel = cmdmsg.velocity
        self.curr_accel = cmdmsg.effort
        self.curr_t = t

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

    #
    # Determine the speed and end position of the thrower for a given target position in xyz space
    #
    def compute_spline(self, target):
        #TODO: To be implemented with a fit to a model.
        # For now, we will just use kinematic equations (once testing of arm is complete)
        pass

        # Note: Add self.TRAJ_TIME to t0 for this spline. Also implement it as a quintic spline that speeds through launch and ends at rest.

    #
    # Callback Function for when we detect a hackysack (somehow through a force sensor) sitting in the scoop.
    #
    def callback_hackysack(self, msg):    
        launch_spline, launch_time = self.compute_spline(msg.target)     # Target being the goal we want to hit. TODO: Figure out how exactly to implement
        
        # Behavior Loop --> From start, go to launch position, then launch, then return to start.
        self.trajectory.add_spline(Goto5(self.curr_t + launch_time + self.TRAJ_TIME, 
                                            self.curr_pos, self.START, self.TRAJ_TIME))
        self.trajectory.add_spline(launch_spline)
        self.trajectory.add_spline(Goto5(self.curr_t, self.curr_pos, self.LAUNCH, self.TRAJ_TIME))

        self.is_waiting = False
        # rospy.loginfo('I heard %s', msg)
       
    #
    # Callback Function for the Error Sensing... How to implement?
    #
    def callback_error(self, msg):
        #TODO: See exactly how we will want to implement this.
        rospy.loginfo('Hello! I heard %s', msg.data)
                            
    
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

    
