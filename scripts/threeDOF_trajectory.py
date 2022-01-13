#!/usr/bin/env python3
#
#   trajectory_generator.py
#
#   Create a continuous stream of joint positions, to be animated.
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import math
import kinematics as kin
import numpy as np

from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5, SineX
from kinematics import p_from_T, q_from_T, R_from_T, T_from_Rp, Rx, Ry, Rz
from sensor_msgs.msg   import JointState
from urdf_parser_py.urdf import Robot


#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher("/joint_states", JointState)
        rospy.sleep(0.25)
        
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')  

        # Initialize the relevant segment parameters
        q_init = np.array([0.0, 0.5, -1.0]).reshape((3,1))
        x_init = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        self.q_prev = q_init
        amplitude = 0.5
        frequency = 1
        self.index = 0
        self.t0    = 0.0
        
        #self.segments = (Hold(q_init, 1.0, 'Joint'),
        #                 SineX(x_init, 1.0, amplitude, frequency, math.inf))
        self.segments = (Hold(q_init, 1.0, 'Joint'))

    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        
        # If the current segment is done, shift to the next.
        if (t - self.t0 >= self.segments[self.index].duration()):
            self.t0    = self.t0 + self.segments[self.index].duration()
            self.index = (self.index+1)
            # If the list were cyclic, you could go back to the start with
            #self.index = (self.index+1) % len(self.segments)
            #self.last_guess = self.reset_guess 
            
        # Check whether we are done with all segments
        if (self.index >= len(self.segments)):
            rospy.signal_shutdown("Done with motion")
            return

        if (self.segments[self.index].space() == 'Joint'):
            # Conducting the multiplicity flip TODO
            (cmdmsg.position, cmdmsg.velocity) = \
            self.segments[self.index].evaluate(t-self.t0)
              
        elif (self.segments[self.index].space() == 'Task'):
            (cart_position, cart_velocity) = \
            self.segments[self.index].evaluate(t-self.t0)
            cmdmsg.position = self.kin.ikin(cart_position, self.q_prev)
            (T, J) = self.kin.fkin(cmdmsg.position)
            cmdmsg.velocity = np.linalg.inv(J[0:3,0:3]) @ cart_velocity
            self.q_prev = cmdmsg.position
        else:
            raise ValueError('Unknown Spline Type')

        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('generator')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

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
        generator.update(t)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()
