#!/usr/bin/env python3

import rospy
import math
import sys
sys.path.insert(1, '/home/me134/me134ws/src/ME134/scripts')
import kinematics as kin
from detector import Detector
from sensor_msgs.msg   import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot
from ME134.msg import array
from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
import numpy as np

'''This code encapsulates the functionality for the receiver arm as well as its
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
            self.curr_spline = self.splines.pop(0)

    #
    # Returns the space of the current trajectory
    #    
    def traj_space(self):
        return self.curr_spline.space()
    
    #
    # Returns the trajectory at the top of the stack
    #
    def pop_spline(self):
        self.curr_spline = self.splines.pop(0)

    #
    # Adds a trajectory to the stack (LIFO)
    # Adds to the end of the list, pop grabs from end of list so added spline is done next
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
#  Receiver Class... For now, it will be implemented as a standalone class (soon to be integrated into Battleship)
#
class Receiver:
    #
    # Initialize.
    #
    def __init__(self):
        # Collect the motor names, which defines the dofs (useful to know)
        self.motors = ['Red/7', 'Red/6', 'Red/1', 'Red/2']
        # self.motors = ['Pan', 'Tilt', 'Elbow', 'Wrist']
        self.dofs = len(self.motors)
        
        # Create a publisher to send the joint commands. 
        # TODO: When moving to battleship, will want to remove these
        # self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        self.pub = rospy.Publisher("/hebi/joint_commands", JointState, queue_size=5)
        rospy.sleep(0.25)
        
        # Create subscribers for the general case and events.
        self.sub = rospy.Subscriber('/hebi/joint_states', JointState, self.callback_actual, queue_size=5)
        

        # Find the starting positions. 
        msg = rospy.wait_for_message('/hebi/joint_states', JointState)
        for i in range(self.dofs):
            if (msg.name[i] != self.motors[i]):
                raise ValueError("Motor names don't match")

        # self.pos_init = np.array([0.0, 0.0, 0.0, 0.0]).reshape((4, 1))

        self.callback_actual(msg)

        # TODO: Create the necessary subscribers for the general case and events.
        # self.sub = rospy.Subscriber('/actual', JointState, self.callback_actual)
        # self.sub_e = rospy.Subscriber('/event', String, self.callback_event)
        # self.sub_hackysack = rospy.Subscriber('/force_sensor', JointState, self.callback_hackysack)
        
        # Grab the robot's URDF from the parameter server.
        # Might have to change this because we'll have multiple URDFs
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'GripperLBracket')

        self.pos_init = np.array(msg.position).reshape((self.dofs, 1))
        [T, J] = self.kin.fkin(self.pos_init[0:3])
        self.pos_init_task = kin.p_from_T(T)

        # Initialize the state of the robot
        #self.curr_pos = np.array([0.0, 0.0, 0.0]).reshape((3, 1))  
        #self.curr_vel = np.array([0.0, 0.0, 0.0]).reshape((3, 1))  
        self.curr_t = 0.0
        self.curr_accel = np.array([0.0, 0.0, 0.0]).reshape((3, 1))
    
        # Initialize the trajectory
        self.START = np.array([-0.10, 0.0, 0.060]).reshape((3, 1))
        self.DROPOFF = np.array([0.63, -0.05, 0.30]).reshape((3, 1))           # TODO: Change to match thrower START
        self.TRAJ_TIME = 5.0        
        self.grasp_open = 0      # 0 is ungrasped, 1 is grasped
        self.grasp_closed = 1

        # Initialize the gravity parameters TODO: Tune and test these parameters for our 4DOF
        self.grav_A = 0.20
        self.grav_B = 4.90
        self.grav_C = 0.0
        self.grav_D = 5.8

        # If we want to float the arm for testing
        self.float = False
        self.TRAVEL_TIME = 15.0

        # HOLD
        self.OFFSET_TIME = 5.0
        self.GRASP_TIME = 5.0

        # Initialize the trajectories that we want for the loop
        hackysack_pos = np.array([-0.25, 0.39, 0.025]).reshape((3, 1))
        q_hackysack = np.array([0.40, 1.06, -2.07]).reshape((3,1))
        q_above_hackysack = np.array([0.38, 1.18, -2.07])
        zero = np.zeros((3,1))
        q_hackysack = hackysack_pos
        z_offset = np.array([0.0, 0.0, 0.05]).reshape((3,1))
        hackysack_above = hackysack_pos + z_offset
        # q_above_hackysack = self.kin.ikin(hackysack_pos + z_offset, np.array([0.40, 1.06, -2.07]).reshape((3,1)))
        self.trajectory = Trajectory([Goto5(self.curr_t, self.pos_init_task[0:3], self.START, self.TRAVEL_TIME,'Task'),              # Actual Pos to Start Pos
            Goto5(self.curr_t+self.TRAVEL_TIME, self.START, hackysack_above, self.TRAVEL_TIME,'Task'),                         # Start Pos to Above Hackysack
            Goto5(self.curr_t+2*self.TRAVEL_TIME, hackysack_above, hackysack_pos, self.OFFSET_TIME,'Task'),                     # Above Hackysack to Hackysack
            Goto5(self.curr_t+2*self.TRAVEL_TIME+self.OFFSET_TIME, hackysack_pos, hackysack_pos, self.GRASP_TIME,'Task'),                           # TODO: IMPLEMENT THE GRASPING PART
            Goto5(self.curr_t+2*self.TRAVEL_TIME+self.OFFSET_TIME+self.GRASP_TIME, hackysack_pos, hackysack_above, self.OFFSET_TIME,'Task'),       #  Hackysack to Above Hackysack
            Goto5(self.curr_t+2*self.TRAVEL_TIME+2*self.OFFSET_TIME+self.GRASP_TIME, hackysack_above, self.DROPOFF, self.TRAVEL_TIME,'Task'),    # Above Hackysack to Thrower Pos
            Goto5(self.curr_t+3*self.TRAVEL_TIME+2*self.OFFSET_TIME+self.GRASP_TIME, self.DROPOFF, self.DROPOFF, self.GRASP_TIME,'Task'),                           # TODO: IMPLEMENT THE GRASPING PART
            Goto5(self.curr_t+3*self.TRAVEL_TIME+2*self.OFFSET_TIME+2*self.GRASP_TIME, self.DROPOFF, self.START, self.TRAVEL_TIME,'Task')])                               # Return to start

        # Initialize any helpful global variables
        self.is_waiting = False

    #
    # Update every 10ms!
    #
    def update(self, t):
        # Create an empty joint state message. TODO: Remove when integrating with battleship.py
        cmdmsg = JointState()
        cmdmsg.name = self.motors

        # CODE FOR TESTING FLOAT
        if self.float:
            # Set q_des = q_actual to make the arm "float"
            nan = float('nan')
            cmdmsg.position = np.array([nan, nan, nan, -np.pi/2 - self.curr_pos[1] - self.curr_pos[2]]).reshape((4,1))
            cmdmsg.velocity = np.array([nan, nan, nan, nan]).reshape((4,1))
            cmdmsg.effort = self.gravity(self.curr_pos)
        
        else:
            # If the current segment is done, shift to the next.
            if (t-self.trajectory.start_time()) >= self.trajectory.duration():
                if self.trajectory.is_empty():
                    self.float = True
                else:
                    self.trajectory.pop_spline()


            # Determine which trajectory and implement functionality
            if (self.trajectory.traj_space() == 'Joint'):
                (this_pos, this_vel) = self.trajectory.update(t)
                
            elif (self.trajectory.traj_space() == 'Task'):
                # TODO: Need to update to account for 4DOF
                (x, xdot) = self.trajectory.update(t)
                cart_pos = np.array(x).reshape((3,1))
                cart_vel = np.array(xdot).reshape((3,1))
                print("pos theta")
                print(self.curr_pos)
                print("cart_pos")
                print(cart_pos)
                #this_pos_tuple = self.kin.ikin(cart_pos, self.curr_pos)

                (T,J) = self.kin.fkin(self.curr_pos)
                p = T[0:3,3:4]
                
                ep = cart_pos - p
                print(ep)

                xrdot = cart_vel + .01*ep
                this_vel = np.linalg.inv(J[0:3,0:3]) @ xrdot
                this_pos = self.curr_pos + dt*this_vel

                #this_pos = np.array([this_pos_tuple[0], this_pos_tuple[1], this_pos_tuple[2]]).reshape((3,1))
                #joint_pos = np.array([cmdmsg.position[0],cmdmsg.position[1],cmdmsg.position[2]]).reshape((3,1))
                #print(joint_pos)
                #(T, J) = self.kin.fkin(this_pos)
                #this_vel = np.linalg.inv(J[0:3,0:3]) @ cart_vel
            else:
                raise ValueError('Unknown Spline Type')
            
        # Print xyz location
        # (T, J) = self.kin.fkin(self.curr_pos)
        # print(T)

        # Update the grasping and the gripper value
        print("pos")
        print(this_pos)
        print("vel")
        print(this_vel)
        cmdmsg.position = np.array([this_pos[0,0], this_pos[1,0], this_pos[2,0], -np.pi/2 - this_pos[1,0] - this_pos[2,0]]).reshape((4,1))
        cmdmsg.velocity = np.array([this_vel[0,0], this_vel[1,0], this_vel[2,0], 0.0 - this_vel[1,0] - this_vel[2,0]]).reshape((4,1))

        # TODO: Implement Gravity Compensation Function for 4DOF
        cmdmsg.effort = self.gravity(self.curr_pos)

        # Store the command message
        self.curr_pos = cmdmsg.position[0:3]
        self.curr_vel = cmdmsg.velocity[0:3]
        self.curr_accel = cmdmsg.effort[0:3]
        self.curr_t = t



        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)
        
    #
    # Gravity Compensation Function
    #
    def gravity(self, pos):
        theta_1 = pos[1]; theta_2 = pos[2]
        tau2 = self.grav_A * math.sin(theta_1 + theta_2) + self.grav_B * math.cos(theta_1 + theta_2)
        tau1 = self.grav_C * math.sin(theta_1) + self.grav_D * math.cos(theta_1) + tau2
        return np.array([0.0, tau1, tau2, 0.0]).reshape((self.dofs,1)) 


    #
    # Callback Function 
    #
    def callback_actual(self, msg):
        # TODO: See what you may need this callback function for.
        self.curr_pos = msg.position[0:3]
        self.curr_vel = msg.velocity[0:3]
       
    #
    # Callback Function for the Error Sensing
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
    rospy.init_node('Receiver')

    # Instantiate the receiver object, encapsulating all
    # the computation and local variables.
    receiver = Receiver()

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
        receiver.update(t)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()

    
