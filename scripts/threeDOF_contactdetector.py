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
        self.sub = rospy.Subscriber('/hebi/joint_states', JointState, self.callback_actual)
        self.sub_e = rospy.Subscriber('/event', String, self.callback_event)
        
        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = kin.Kinematics(robot, 'world', 'tip')

        # Initialize the relevant joint and position parameters
        self.x_init = np.array([0.0, 0.35, 0.10]).reshape((3,1))
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
        
        # Define values for the actual motor positions
        self.act_pos = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        self.act_vel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        
        # Define the explicit value of the sinusoidal function
        self.amplitude = 0.15
        self.frequency = 1.0
        self.start_t = 0.0
        
        # Indicate without an explicit event how long to hold in initial state
        self.hold   = True
        self.hold_time = 4.0
        
        # Indicate without an explicit event how long to perform the joint flip
        self.flip = False
        self.flip_time = 2 * math.pi / self.frequency
        self.flip_moment = 0.0
        self.flip_count = 0
        
        # Initialize with holding trajectory
        # For future, we will want to implement this as a stack of trajectories
        #self.trajectory = Trajectory(Hold(self.curr_t, self.curr_pos, self.hold_time, 'Joint'))
        # Trajectory from the position of the arm received to q_init
        self.trajectory = Trajectory(QuinticSpline(self.curr_t, self.initpos, self.curr_vel, self.curr_accel, self.curr_pos, self.curr_vel, self.curr_accel, self.hold_time, 'Joint'))


    # Update every 10ms!
    def update(self, t):
        # Create an empty joint state message.
        cmdmsg = JointState()
        cmdmsg.name = self.motors
            
        # Change from holding to the sinusoidal trajectory
        if (self.hold):
            if (t >= self.hold_time):
                self.start_t = t
                self.trajectory = \
                Trajectory(SineX(self.x_init, self.start_t, self.amplitude, self.frequency, math.inf))
                self.hold = False
                
        # Change from flipping to the sinusoidal trajectory
        if (self.flip):
            if (t >= self.flip_moment + self.flip_time):
                self.trajectory = \
                Trajectory(SineX(self.x_init, self.start_t, self.amplitude, self.frequency, math.inf))
                self.flip = False
        
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
        
        for i in range(self.dofs):
            if (msg.name[i] != self.motors[i]):
                raise ValueError("Motor names don't match")
        self.act_pos = np.array(msg.position).reshape((3,1))
        self.act_vel = np.array(msg.velocity).reshape((3,1))
        #rospy.loginfo('I heard %s', msg)
        posthreshold = 0.02
        if np.any(np.abs(self.act_pos - self.curr_pos) > posthreshold):
            self.contactdetected()
       
        # Contact Detector
    def contactdetected(self):
        #TODO
        rospy.logerr('contact detected')
        rospy.logerr(np.abs(self.act_pos - self.curr_pos))
       
    # Callback Function for the Event    
    def callback_event(self, msg):
        rospy.loginfo('Hello! I heard %s', msg.data)
        
        # Update values for the update function
        self.flip = True
        self.flip_moment = self.curr_t
        self.flip_count += 1
        
        # Joint angle corresponding to elbow-down, flipped multiplicity
        if (self.flip_count % 2 == 1):  # Odd number of flips, go counterclockwise
            q1 = math.pi + self.curr_pos[0]               
            q2 = math.pi - self.curr_pos[1]               
            q3 = -1 * self.curr_pos[2]              
        else:                           # Even number of flips, go clockwise
            q1 = self.curr_pos[0] - math.pi
            q2 = math.pi - self.curr_pos[1]
            q3 = -1 * self.curr_pos[2]

        joint_flip = np.array([q1, q2, q3]).reshape((3,1))

        # Joint velocities corresponding to elbow down, flipped multiplicity
        qdot1 = self.curr_vel[0]
        qdot2 = -1*self.curr_vel[1]
        qdot3 = -1*self.curr_vel[2]

        joint_flip_dot = np.array([qdot1, qdot2, qdot3]).reshape((3,1))

        # self.trajectory = Trajectory(Goto5(self.curr_t, self.curr_pos, joint_flip, self.flip_time, 'Joint'))
        self.trajectory = Trajectory(QuinticSpline(self.curr_t, self.curr_pos, self.curr_vel, self.curr_accel,
                            joint_flip, joint_flip_dot, self.curr_accel, self.flip_time, 'Joint'))
                            
    

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
