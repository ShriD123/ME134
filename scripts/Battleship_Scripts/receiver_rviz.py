#!/usr/bin/env python3

import rospy
import math
import sys
# sys.path.insert(1, '/home/me134/me134ws/src/ME134/scripts')
sys.path.insert(1, '/home/robot/133ws/src/ME134/scripts')
import kinematics as kin
# from detector import Detector
from sensor_msgs.msg   import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot
# from ME134.msg import aruco_center
from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
import numpy as np
import time

'''This code encapsulates the functionality for the receiver arm as well as its
corresponding error sensing and the necessary subscribers for itself.'''


###############################################################################
#
#  Receiver Trajectory Class --> Changed to allow for appropriate error sensing.
#
class Goto5Trajectory:
    #
    # Initialize
    #
    def __init__(self, list_splines=[]):
        # Each list element should pass in p0, pf, T, and space... t0 will be automatically determined
        self.splines = list_splines
        if len(list_splines) == 0:
            self.curr_spline = None
        else:
            params = self.splines.pop()
            self.curr_spline = Goto5(0.0, params[0], params[1], params[2], params[3])

    #
    # Returns the space of the current trajectory
    #    
    def traj_space(self):
        return self.curr_spline.space()
    
    #
    # Returns the trajectory at the top of the stack
    #
    def pop_spline(self, curr_t):
        params = self.splines.pop()
        self.curr_spline = Goto5(curr_t, params[0], params[1], params[2], params[3])

    #
    # Adds a trajectory to the stack (LIFO)
    # Adds to the end of the list, pop grabs from end of list so added spline is done next
    def add_spline(self, next_spline):
        # Each list element should pass in p0, pf, T, and space... t0 will be automatically determined
        if type(next_spline) is not list:
            raise ValueError('Input spline should be a list of the Goto5 parameters.')
        elif len(next_spline) != 4:
            raise ValueError('Incorrect number of parameters provided for Goto5 spline.')
        else:
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
    def __init__(self, init_pos):

        # Collect the motor names, which defines the dofs (useful to know)
        # self.motors = ['Red/7', 'Red/6', 'Red/4', 'Red/2', 'Red/3']
        self.motors = ['Pan', 'Tilt', 'Elbow', 'Wrist']
        self.dofs = len(self.motors)
        
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')

        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)


        self.pos_init = np.array(init_pos).reshape((self.dofs, 1))
        [T, J] = self.kin.fkin(self.pos_init[0:3])
        self.pos_init_task = kin.p_from_T(T)

        # Initialize the state of the robot
        self.curr_pos = self.pos_init[0:3]
        self.curr_vel = np.array([0.0, 0.0, 0.0]).reshape((3, 1))  
        self.curr_t = 0.0
        self.curr_accel = np.array([0.0, 0.0, 0.0]).reshape((3, 1))
    
        # Initialize the trajectory
        self.START = np.array([-0.50, 0.0, 0.3 + 0.12]).reshape((3, 1))
        self.START_joint = np.array(self.kin.ikin(self.START, np.array([1.69, 1.41, -1.56]).reshape((3,1)))).reshape((3,1))

        self.WAIT = np.array([0.50, -0.2, 0.3 + 0.12]).reshape((3, 1))

        self.DROPOFF_ABOVE = np.array([0.50, -0.09, 0.42]).reshape((3, 1))
        self.DROPOFF_ABOVE_joint = np.array(self.kin.ikin(self.DROPOFF_ABOVE, np.array([-1.56, 1.20, -1.22]).reshape((3,1)))).reshape((3, 1))
        self.gripper_theta = 0.0
        self.INIT_TIME = 3.0

        # Initialize the gravity parameters
        self.grav_A = 0.20
        self.grav_B = 4.90
        self.grav_C = 0.0
        self.grav_D = 5.8

        # If we want to float the arm for testing
        self.TRAVEL_TIME = 5.0

        # HOLD
        self.OFFSET_TIME = 1
        self.GRASP_TIME = 0.5

        # Populate the trajectory
        # self.compute_spline()

        self.hackysack_pos = np.array([0.25, 0.25, 0.0]).reshape((3,1))
        z_offset = np.array([0.0, 0.0, 0.0 + 0.12]).reshape((3,1))
        hackysack_above = self.hackysack_pos + z_offset

        # # For testing the new method of adding a spline
        # self.trajectory = Goto5Trajectory([[self.START_joint, self.START_joint, 1.0, 'Joint']])
        # self.trajectory.add_spline([self.hackysack_pos, self.DROPOFF_ABOVE, self.TRAVEL_TIME, 'Task'])
        # self.trajectory.add_spline([hackysack_above, self.hackysack_pos, self.OFFSET_TIME, 'Task'])
        # self.trajectory.add_spline([self.START, hackysack_above, self.TRAVEL_TIME, 'Task'])
        
        

        self.trajectory = Goto5Trajectory([
                # Returns to start from dropoff point
            [self.DROPOFF_ABOVE_joint, self.START_joint, self.TRAVEL_TIME,'Joint'],
                # Ungrasps the hackysack
                # Above hackysack to dropoff point
            [hackysack_above, self.DROPOFF_ABOVE, self.TRAVEL_TIME,'Task'],   
                # Hackysack to Above hackysack
            [self.hackysack_pos, hackysack_above, self.OFFSET_TIME,'Task'],
                # Grasps the hackysack
                # Above hackysack to hackysack
            [hackysack_above, self.hackysack_pos, self.OFFSET_TIME,'Task'], 
                # Start Position to Above hackysack
            [self.START, hackysack_above, self.TRAVEL_TIME,'Task'],
                # Move from actual position to start position
                # Move from actual position to start position
            [self.pos_init_task[0:3], self.pos_init_task[0:3], self.INIT_TIME,'Task']])
        self.INIT_TIME = 0.1   


        # Initialize any helpful global variables
        self.is_waiting = False

        self.msg_sent = False

    #
    # Update every 10ms!
    #
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = self.motors

        # If the current segment is done, shift to the next.
        if (t-self.trajectory.start_time()) >= self.trajectory.duration():
            if self.trajectory.is_empty():
                self.is_waiting = True
                if not self.msg_sent:
                    message = "Throw"
                    # self.rtpub.publish(message)
                    self.msg_sent = True
            else:
                self.trajectory.pop_spline(t)


        if self.is_waiting:
            start_tuple = self.kin.ikin(self.WAIT, np.array(self.curr_pos).reshape((3,1)))
            start_array = np.array(start_tuple).reshape((3,1))
            cmdmsg.position = np.array([start_array[0,0], start_array[1,0], start_array[2,0], -np.pi/2 - start_array[1,0] - start_array[2,0]]).reshape((4,1))
            cmdmsg.velocity = np.array([0.0, 0.0, 0.0, 0.0]).reshape((4, 1))

            self.pos_init_task = self.START

        else:
            # Determine which trajectory and implement functionality
            if (self.trajectory.traj_space() == 'Joint'):
                (this_pos, this_vel) = self.trajectory.update(t)
                
            elif (self.trajectory.traj_space() == 'Task' or self.trajectory.traj_space() == 'GripOn' or self.trajectory.traj_space() == 'GripOff'):
                (x, xdot) = self.trajectory.update(t)
                cart_pos = np.array(x).reshape((3,1))
                cart_vel = np.array(xdot).reshape((3,1))
                this_pos_tuple = self.kin.ikin(cart_pos, np.array(self.curr_pos).reshape((3,1)))

                this_pos = np.array(this_pos_tuple).reshape((3,1))
                (T, J) = self.kin.fkin(this_pos)
                this_vel = np.linalg.inv(J[0:3,0:3]) @ cart_vel
                if (self.trajectory.traj_space() == 'GripOn'):
                    self.gripper_theta = -0.55
                if (self.trajectory.traj_space() == 'GripOff'):
                    self.gripper_theta = 0.0
            
            else:

                raise ValueError('Unknown Spline Type')


            cmdmsg.position = np.array([this_pos[0,0], this_pos[1,0], this_pos[2,0], -np.pi/2 - this_pos[1,0] - this_pos[2,0]]).reshape((4,1))
            cmdmsg.velocity = np.array([this_vel[0,0], this_vel[1,0], this_vel[2,0], 0.0 - this_vel[1,0] - this_vel[2,0]]).reshape((4,1))
        if (cmdmsg.position[0] - self.curr_pos[0] > 0.2 or cmdmsg.position[1] - self.curr_pos[1] > 0.2 or cmdmsg.position[2] - self.curr_pos[2] > 0.2):
            rospy.logerr("Bad theta input")
            print(cmdmsg.position)
            # rospy.signal_shutdown()

        # Store the command message
        self.curr_pos = cmdmsg.position[0:3]
        self.curr_vel = cmdmsg.velocity[0:3]
        self.curr_t = t

        # Publish the command message
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
    # Callback function to exit waiting condition and go into trajectory
    #
    def callback_exitwait(self, msg):
        rospy.loginfo('Hello! I heard %s', msg.data)
        # Exit wait if wait is true
        print(self.is_waiting)
        if self.is_waiting:
            # Populate spline with new trajectory
            # self.compute_spline()
            self.is_waiting = False
            self.msg_sent = False
            print("Done Computing Splines")
            
    def compute_spline(self):
        # Initialize the trajectories that we want for the loop
        xy_sackpos = rospy.wait_for_message('/blob_loc', aruco_center)
        sack_found = False
        #print(xy_sackpos)
        while xy_sackpos.blob == "No Blob":
            xy_sackpos = rospy.wait_for_message('/blob_loc', aruco_center)
            print("no blob on detector range")
        while not sack_found:
            xy_sackpos = rospy.wait_for_message('/blob_loc', aruco_center)
            for ind,locx in enumerate(xy_sackpos.datax):
                locy = xy_sackpos.datay[ind]
                if (locx < 0.57 and locx > - 0.64 and locy > 0.10 and locy < 0.74):
                    self.hackysack_pos = np.array([xy_sackpos.datax[ind] - 0.05, xy_sackpos.datay[ind], 0.025 + 0.12]).reshape((3, 1))
                    sack_found = True
                    print("sack found")
                    break
            if not sack_found:
                print("all sacks out of receiver reach")

        #self.hackysack_pos = curr_sack
        z_offset = np.array([0.0, 0.0, 0.0 + 0.12]).reshape((3,1))
        hackysack_above = self.hackysack_pos + z_offset
        # hackysack_above_joint = np.array(self.kin.ikin(hackysack_above, np.array([0.9, 1.2, -1.8]).reshape((3,1)))).reshape((3, 1))
                        # Actual Pos to Start Pos
        if self.INIT_TIME > 0.0:
            self.trajectory = Trajectory([Goto5(self.curr_t, self.pos_init_task[0:3], self.START, self.INIT_TIME,'GripOff'),
                            # Start Pos to Above Hackysack              
                Goto5(self.curr_t+self.INIT_TIME, self.START, hackysack_above, self.TRAVEL_TIME,'Task'),         
                            # Above Hackysack to Hackysack
                Goto5(self.curr_t+self.INIT_TIME+self.TRAVEL_TIME, hackysack_above, self.hackysack_pos, self.OFFSET_TIME,'Task'),        
                            # Grasp the hackysack
                Goto5(self.curr_t+self.INIT_TIME+self.TRAVEL_TIME+self.OFFSET_TIME, self.hackysack_pos, self.hackysack_pos, self.GRASP_TIME,'GripOn'),    
                            # Hackysack to Above Hackysack
                Goto5(self.curr_t+self.INIT_TIME+self.TRAVEL_TIME+self.OFFSET_TIME+self.GRASP_TIME, self.hackysack_pos, hackysack_above, self.OFFSET_TIME,'Task'),  
                            # Above Hackysack to Thrower Pos
                Goto5(self.curr_t+self.INIT_TIME+self.TRAVEL_TIME+2*self.OFFSET_TIME+self.GRASP_TIME, hackysack_above, self.DROPOFF_ABOVE, self.TRAVEL_TIME,'Task'),   
                            #  
                Goto5(self.curr_t+self.INIT_TIME+2*self.TRAVEL_TIME+2*self.OFFSET_TIME+self.GRASP_TIME, self.DROPOFF_ABOVE, self.DROPOFF_ABOVE, self.GRASP_TIME,'GripOff'),
                Goto5(self.curr_t+self.INIT_TIME+2*self.TRAVEL_TIME+2*self.OFFSET_TIME+2*self.GRASP_TIME, self.DROPOFF_ABOVE, self.WAIT, self.OFFSET_TIME,'Task')])                               # Return to start
        else:
            self.trajectory = Trajectory([Goto5(self.curr_t, self.WAIT, hackysack_above, self.TRAVEL_TIME,'Task'),         
                            # Above Hackysack to Hackysack
                Goto5(self.curr_t+self.TRAVEL_TIME, hackysack_above, self.hackysack_pos, self.OFFSET_TIME,'Task'),        
                            # Grasp the hackysack
                Goto5(self.curr_t+self.TRAVEL_TIME+self.OFFSET_TIME, self.hackysack_pos, self.hackysack_pos, self.GRASP_TIME,'GripOn'),    
                            # Hackysack to Above Hackysack
                Goto5(self.curr_t+self.TRAVEL_TIME+self.OFFSET_TIME+self.GRASP_TIME, self.hackysack_pos, hackysack_above, self.OFFSET_TIME,'Task'),  
                            # Above Hackysack to Thrower Pos
                Goto5(self.curr_t+self.TRAVEL_TIME+2*self.OFFSET_TIME+self.GRASP_TIME, hackysack_above, self.DROPOFF_ABOVE, self.TRAVEL_TIME,'Task'),   
                            #  
                Goto5(self.curr_t+2*self.TRAVEL_TIME+2*self.OFFSET_TIME+self.GRASP_TIME, self.DROPOFF_ABOVE, self.DROPOFF_ABOVE, self.GRASP_TIME,'GripOff'),
                Goto5(self.curr_t+2*self.TRAVEL_TIME+2*self.OFFSET_TIME+2*self.GRASP_TIME, self.DROPOFF_ABOVE, self.WAIT, self.TRAVEL_TIME,'Task')])                               # Return to start
      
        self.INIT_TIME = 0.0                 
    
###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('Receiver')
    receiver = Receiver(np.array([-0.5, 0.0, 0.5, 0.0]).reshape((4,1)))

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                (dt, rate))

    time.sleep(1)

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

    
