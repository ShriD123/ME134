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
from std_msgs.msg import String
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
        # Collect the motor names, which defines the dofs (useful to know)
        #self.motors = ['theta1', 'theta2', 'theta3']
        # Names of the motors
        self.motors = ['Red/1', 'Red/2', 'Red/3']
        self.dofs = len(self.motors)
        
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        # self.pub = rospy.Publisher("/joint_states", JointState, queue_size=5)
        self.pub = rospy.Publisher("/hebi/joint_commands", JointState, queue_size=5)
        rospy.sleep(0.25)
        
        # IMPLEMENT FINDING THE STARTING POSITIONS LATER AND THEN MOVE TO ZERO STARTING OUT
        
        # Find the starting positions.  This will block, but that's
        # appropriate as we don't want to start until we have this
        # information.
        msg = rospy.wait_for_message('/hebi/joint_states', JointState);
        for i in range(self.dofs):
            if (msg.name[i] != self.motors[i]):
                raise ValueError("Motor names don't match")

        self.initpos = np.array(msg.position).reshape((3, 1))
        for i in range(self.dofs):
            rospy.loginfo("Starting motor[%d] '%s' at pos: %f rad",
                          i, self.motors[i], self.initpos[i])

        # Create subscribers for the general case and events.
        self.sub = rospy.Subscriber('/actual', JointState, self.callback_actual)
        self.sub_e = rospy.Subscriber('/event', String, self.callback_event)
        
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')

        # Initialize the relevant joint and position parameters
        
        # DESIRED POSITION
        x_des = np.array([0.0, 0.20, 0.08]).reshape((3,1))
        
        x_comp = np.array([0.0, 0.0, 0.015]).reshape((3, 1))
        self.x_init = x_des + x_comp
        
        
        q_guess = np.array([0.0, 0.5, 1.0]).reshape((3,1))
        
        # Actually want to use the actual values in the future.
        # Initialize the state of the robot
        self.q_init = self.kin.ikin(self.x_init, q_guess)
        # (T_i, J_i) = self.kin.fkin(self.q_init)
        # self.x_init_act = p_from_T(T_i)

        self.curr_pos = self.q_init
        self.curr_vel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        self.curr_t = 0.0
        self.curr_accel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        
        
        # Indicate without an explicit event how long to hold in initial state
        self.hold   = True
        self.hold_time = 4.0
        
        
        # Initialize with holding trajectory

        # Trajectory from the position of the arm received to q_init
        self.trajectory = Trajectory(QuinticSpline(self.curr_t, self.initpos, self.curr_vel, self.curr_accel, self.curr_pos, self.curr_vel, self.curr_accel, self.hold_time, 'Joint'))


    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = self.motors

        
        # Determine which trajectory and implement functionality
        if (self.trajectory.traj_space() == 'Joint'):
            (cmdmsg.position, cmdmsg.velocity) = self.trajectory.update(t)
              
        elif (self.trajectory.traj_space() == 'Task'):
            (x, xdot) = self.trajectory.update(t)
            cart_pos = np.array(x).reshape((3,1))
            cart_vel = np.array(xdot).reshape((3,1))
            
            cmdmsg.position = self.kin.ikin(cart_pos, self.curr_pos)
            (T, J) = self.kin.fkin(cmdmsg.position)
            cmdmsg.velocity = np.linalg.inv(J[0:3,0:3]) @ cart_vel
        else:
            raise ValueError('Unknown Spline Type')
        
        if (t >= self.hold_time):
            self.trajectory = Trajectory(Stay(self.hold_time, self.curr_pos))
            
            
        # No gravity compensation yet so 0 effort for now
        cmdmsg.effort = [0.0, 0.0, 0.0]

        # Store the command message
        self.curr_pos = cmdmsg.position
        self.curr_vel = cmdmsg.velocity
        self.curr_t = t

        # Send the command (with the current time).
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)
        
    
    # Callback Function 
    def callback_actual(self, msg):
        # Future Function to Implement to account for Gravity Compensation
        # Also should read the initial joint pos of the motors to initialize the trajectory
        rospy.loginfo('I heard %s', msg)
       
       
    # Callback Function for the Event    
    def callback_event(self, msg):
        rospy.loginfo('Hello! I heard %s', msg.data)
        
        
    

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
