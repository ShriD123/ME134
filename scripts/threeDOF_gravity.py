#!/usr/bin/env python3
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import math
import kinematics as kin
import numpy as np
import std_msgs

from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
from kinematics import p_from_T, q_from_T, R_from_T, T_from_Rp, Rx, Ry, Rz
from sensor_msgs.msg   import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot
from ME134.msg import array


###############################################################################
#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Collect the motor names, which defines the dofs (useful to know)
        self.motors = ['Red/1', 'Red/2', 'Red/3']
        self.dofs = len(self.motors)
        
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher('/hebi/joint_commands', JointState, queue_size=5)
        rospy.sleep(0.25)

        # Create subscribers for the general case and events.
        self.sub = rospy.Subscriber('/hebi/joint_states', JointState, self.callback_actual, queue_size=5)
        self.sub_tune = rospy.Subscriber('/tune', String, self.callback_tune, queue_size=5)
        # self.sub_num = rospy.Subscriber('/number', std_msgs.msg.Float32, self.callback_number, queue_size=5)
        self.sub_num = rospy.Subscriber('/number', array, self.callback_number, queue_size=5)
        
        
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')

        # Initialize the joints using actual positions of the motors.
        msg = rospy.wait_for_message('hebi/joint_states', JointState);
        for i in range(self.dofs):
            if (msg.name[i] != self.motors[i]):
                raise ValueError("Motor names don't match")

        self.q_init = np.array(msg.position).reshape((3, 1))
        for i in range(self.dofs):
            rospy.loginfo("Starting motor[%d] '%s' at pos: %f rad",
                          i, self.motors[i], self.q_init[i])
        self.callback_actual(msg)

        self.curr_pos = self.q_init
        self.curr_vel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        self.curr_t = 0.0
        self.curr_accel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        
        # Decide what you want to do
        self.float = True               # Testing the floating ability of the arm
        self.tune = False               # Tune the robot by hand
        
        # Gravity Constants
        self.A = -0.07
        self.B = 0.04
        self.C = -0.04
        self.D = 1.15


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
        
        if self.tune:
            # TODO: Implement this a lil later
            pass

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
        # To call this, do rostopic pub -1 /tune ME134/array [a,b,c,d] (no spaces)
        rospy.loginfo('I heard %s', msg)
        data = msg.data
        if len(data) != 4:
            raise ValueError("Gravity Params don't match")
        self.A = data[0]
        self.B = data[1]
        self.C = data[2]
        self.D = data[3]
        
    # Callback Function for tuning the gravity parameters
    def callback_number(self, msg):
        self.A = msg.data[0]


    # Gravity Compensation Function
    def gravity(self, pos):
        theta_1 = pos[1]; theta_2 = pos[2]
        tau2 = self.A * math.sin(theta_1 + theta_2) + self.B * math.cos(theta_1 + theta_2)
        tau1 = self.C * math.sin(theta_1) + self.D * math.cos(theta_1) + tau2
        return np.array([0, tau1, -1*tau2]).reshape((3,1)) 


###############################################################################
#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('demo')

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
