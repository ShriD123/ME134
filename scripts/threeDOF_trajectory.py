#!/usr/bin/env python3
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


###############################################################################
#
#  Trajectory Class
#
class Trajectory:
    # Initialize
    def __init__(self, spline):
        self.spline = spline
        self.space = spline.space()
        
    def traj_space(self):
        return self.space
    
    def spline(self):
        return self.spline
    
    def update(self, t):
        (a, b) = self.spline.evaluate(t)
        return (a, b)


###############################################################################
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
        
        # Create subscribers for the general case and events.
        # self.sub = rospy.Subscriber('/actual', Type?, self.callback(cmdmsg))
        # self.sub_e = rospy.Subscriber('/event', Type?, self.callback_event(cmdmsg))
        
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')  

        # Initialize the relevant joint and position parameters
        self.x_init = np.array([0.0, 1.25, 0.01]).reshape((3,1))
        q_guess = np.array([0.0, 0.5, -1.0]).reshape((3,1))
        self.q_init = self.kin.ikin(self.x_init, q_guess)
        self.q_prev = self.q_init
        
        # Define the explicit value of the sinusoidal function
        self.amplitude = 0.25
        self.frequency = 1.0
        
        # Indicate without an explicit event how long to hold in initial state
        self.hold   = True                                            
        self.holdTime = 1.0
        
        # Initialize with holding trajectory
        self.trajectory = Trajectory(Hold(self.q_init, 1.0, 'Joint'))


    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = ['theta1', 'theta2', 'theta3']
            
        # Change from holding to the sinusoidal trajectory
        if (self.hold):
            if (t >= self.holdTime):
                self.trajectory = \
                Trajectory(SineX(self.x_init, t, self.amplitude, self.frequency, math.inf))
                self.hold = False
        
        # Determine which trajectory and implement functionality
        if (self.trajectory.traj_space() == 'Joint'):
            (cmdmsg.position, cmdmsg.velocity) = self.trajectory.update(t)
              
        elif (self.trajectory.traj_space() == 'Task'):
            (x, xdot) = self.trajectory.update(t)
            cart_pos = np.array(x).reshape((3,1))
            cart_vel = np.array(xdot).reshape((3,1))
            
            cmdmsg.position = self.kin.ikin(cart_pos, self.q_prev)
            (T, J) = self.kin.fkin(cmdmsg.position)
            cmdmsg.velocity = np.linalg.inv(J[0:3,0:3]) @ cart_vel
            self.q_prev = cmdmsg.position
        else:
            raise ValueError('Unknown Spline Type')

        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)
        
    
    # Callback Function (General used for what?)
    def callback(self, msg):
        # How do I code the rospy listener?
        # Also where exactly do we call this function?
        rospy.loginfo('I heard %s', msg)
       
       
    # Callback Function for the Event    
    def callback_event(self, msg):
        # How do I code the rospy listener?
        # Also where exactly do we call this function?
        
        # Extract relevant state parameters from the msg
        position = msg[0]
        velocity = msg[1]
        
        intermediate_position = np.array([0.0, 0.0, 0.0]) #TODO
        intermediate_velocity = np.array([0.0, 0.0, 0.0]) #TODO
        
        
        # Change the trajectory to a joint spline given the current state
        self.trajectory = Trajectory(Goto(position, intermediate_position, 5.0, 'Joint'),
                                     Goto(intermediate_position, intermediate_position, 5.0, 'Joint'))
        # NEED TO ADJUST THE CODE FOR THE TRAJECTORY CLASS TO HANDLE MULTIPLE SPLINES INSIDE IT
        # ALSO NEED TO DETERMINE HOW TO ENSURE IT PASSES MULTIPLICITY AND DOESN'T JUST COME BACK... probably Quintic spline
    
        
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
