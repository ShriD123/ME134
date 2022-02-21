#!/usr/bin/env python3

import rospy
import math
import kinematics as kin
import splines
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
        self.motors = ['Red/1', 'Red/2', 'Red/3', 'Red/4']
        self.dofs = len(self.motors)
        
        # Create a publisher to send the joint commands. 
        #TODO: When moving to battleship, will want to remove these
        # self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        self.pub = rospy.Publisher("/hebi/joint_commands", JointState, queue_size=5)
        rospy.sleep(0.25)
        
        # Find the starting positions. 
        msg = rospy.wait_for_message('/hebi/joint_states', JointState)
        for i in range(self.dofs):
            if (msg.name[i] != self.motors[i]):
                raise ValueError("Motor names don't match")
        self.pos_init = np.array(msg.position).reshape((self.dofs, 1))

        # TODO: Create the necessary subscribers for the general case and events.
        # self.sub = rospy.Subscriber('/actual', JointState, self.callback_actual)
        # self.sub_e = rospy.Subscriber('/event', String, self.callback_event)
        
        # Grab the robot's URDF from the parameter server.
        # Might have to change this because we'll have multiple URDFs
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')

        # Initialize the state of the robot
        self.curr_pos = self.pos_init
        self.curr_vel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        self.curr_t = 0.0
        self.curr_accel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
    
        
        # Initialize with holding trajectory
        # TODO: Will need to implement this as a stack of trajectories
        # TODO: Initialize to some starting position given the actual position
        self.trajectory = Trajectory(Hold(self.curr_t, self.curr_pos, self.hold_time, 'Joint'))

    #
    # Update every 10ms!
    #
    def update(self, t):
        # Create an empty joint state message. TODO: Remove when integrating with battleship.p
        cmdmsg = JointState()
        cmdmsg.name = self.motors
        
        # Determine which trajectory and implement functionality
        if (self.trajectory.traj_space() == 'Joint'):
            # TODO: Need to update to account for 4DOF
            (cmdmsg.position, cmdmsg.velocity) = self.trajectory.update(t)
              
        elif (self.trajectory.traj_space() == 'Task'):
            # TODO: Need to update to account for 4DOF
            (x, xdot) = self.trajectory.update(t)
            cart_pos = np.array(x).reshape((self.dofs,1))
            cart_vel = np.array(xdot).reshape((self.dofs,1))
            
            cmdmsg.position = self.kin.ikin(cart_pos, self.curr_pos)
            (T, J) = self.kin.fkin(cmdmsg.position)
            cmdmsg.velocity = np.linalg.inv(J[0:3,0:3]) @ cart_vel
        else:
            raise ValueError('Unknown Spline Type')
            
        # TODO: Implement Gravity Compensation
        cmdmsg.effort = self.gravity(self.curr_pos)

        # Store the command message
        self.curr_pos = cmdmsg.position
        self.curr_vel = cmdmsg.velocity
        self.curr_accel = cmdmsg.effort
        self.curr_t = t

        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)
        
    #
    # Gravity Compensation Function
    #
    def gravity(self, pos):
        # TODO: Need to update to account for 4DOF
        theta_1 = pos[1]; theta_2 = pos[2]
        tau2 = self.A * math.sin(theta_1 + theta_2) + self.B * math.cos(theta_1 + theta_2)
        tau1 = self.C * math.sin(theta_1) + self.D * math.cos(theta_1) + tau2
        return np.array([0, tau1, -1*tau2]).reshape((self.dofs,1)) 

    #
    # Move to a specified position.
    #
    def move(self, pos):
        # TODO: Need to implement since this function will be used in multiple places.
        pass

    #
    # Callback Function 
    #
    def callback_actual(self, msg):
        # TODO: See what you may need this callback function for.
        rospy.loginfo('I heard %s', msg)
       
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

    
