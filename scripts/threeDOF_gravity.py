#!/usr/bin/env python3
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import math
import kinematics as kin
import numpy as np

from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
from kinematics import p_from_T, q_from_T, R_from_T, T_from_Rp, Rx, Ry, Rz
from sensor_msgs.msg   import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot


###############################################################################
#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Collect the motor names, which defines the dofs (useful to know)
        self.motors = ['theta1', 'theta2', 'theta3']
        self.dofs = len(self.motors)
        
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        rospy.sleep(0.25)

        # Create subscribers for the general case and events.
        self.sub = rospy.Subscriber('/hebi/joint_commands', JointState, self.callback_actual, queue_size=1)
        self.sub_tune = rospy.Subscriber('/tune', String, self.callback_tune)
        
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')

        # Initialize the joints using actual positions of the motors.
        msg = rospy.wait_for_message('hebi/joint_commands', JointState);
        self.callback_actual(msg)

        self.q_init = msg.position              # To start the trajectories

        self.curr_pos = self.q_init
        self.curr_vel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        self.curr_t = 0.0
        self.curr_accel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        
        # Decide what you want to do
        self.float = True               # Testing the floating ability of the arm
        self.tune = False               # Tune the robot by hand


    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = self.motors

        if self.float:
            # Set q_des = q_actual to make the arm "float"
            nan = float('nan')
            cmdmsg.position = np.array([nan, nan, nan]).reshape((3,1))
            cmdmsg.velocity = np.array([nan, nan, nan]).reshape((3,1))
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
       
       
    # Callback Function for the
    def callback_tune(self, msg):
        # TODO Add this, potentially for tuning the parameters
        pass


    # Gravity Compensation Function
    def gravity(self, pos):
        theta_1 = pos[1]; theta_2 = pos[2]
        tau2 = self.A * math.sin(theta_1 + theta_2) + self.B * math.cos(theta_1 + theta_2)
        tau1 = self.C * math.sin(theta_1) + self.D * math.cos(theta_1) + tau2
        return (0, tau1, tau2) 


###############################################################################
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
