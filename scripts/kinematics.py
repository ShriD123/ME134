#!/usr/bin/env python3
#
#   kinematics.py
#
#   Kinematics Class and Helper Functions
#
#   This computes the forward kinematics and Jacobian using the
#   kinematic chain.  It also includes test code when run
#   independently.
#
import rospy
import math
import numpy as np

from urdf_parser_py.urdf import Robot


#
#  Kinematics Helper Functions
#
#  These compute
#    3x1 point vectors "p"
#    3x1 axes of rotation "e"
#    3x3 rotation matrices "R"
#    1x4 quaternions "q"
#    4x4 transforms "T"
#  from each other or
#    1x3 xyz vector of positions
#    1x3 rpy vector of angles
#    1x6 xyz/rpy origin
#    1x3 axis vector
#
# Points:
def p_from_T(T):
    return T[0:3,3:4]

def p_from_xyz(xyz):
    return np.array(xyz).reshape((3,1))

# Axes:
def e_from_axis(axis):
    return np.array(axis).reshape((3,1))

# Rotation Matrices:
def R_from_T(T):
    return T[0:3,0:3]

def R_from_rpy(rpy):
    return Rz(rpy[2]) @ Ry(rpy[1]) @ Rx(rpy[0])

def Rx(theta):
    c = np.cos(theta);
    s = np.sin(theta);
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
def Ry(theta):
    c = np.cos(theta);
    s = np.sin(theta);
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
def Rz(theta):
    c = np.cos(theta);
    s = np.sin(theta);
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

def R_from_axisangle(axis, theta):
    ex = np.array([[     0.0, -axis[2],  axis[1]],
                   [ axis[2],      0.0, -axis[0]],
                   [-axis[1],  axis[0],     0.0]])
    return np.eye(3) + np.sin(theta) * ex + (1.0-np.cos(theta)) * ex @ ex

def R_from_q(q):
    norm2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
    return - np.eye(3) + (2/norm2) * (
      np.array([[q[1]*q[1]+q[0]*q[0],q[1]*q[2]-q[0]*q[3],q[1]*q[3]+q[0]*q[2]],
                [q[2]*q[1]+q[0]*q[3],q[2]*q[2]+q[0]*q[0],q[2]*q[3]-q[0]*q[1]],
                [q[3]*q[1]-q[0]*q[2],q[3]*q[2]+q[0]*q[1],q[3]*q[3]+q[0]*q[0]]]))

# Quaternions:
def q_from_T(T):
    return q_from_R(R_from_T(T))

def q_from_R(R):
    A = [1.0 + R[0][0] + R[1][1] + R[2][2],
         1.0 + R[0][0] - R[1][1] - R[2][2],
         1.0 - R[0][0] + R[1][1] - R[2][2],
         1.0 - R[0][0] - R[1][1] + R[2][2]]
    i = A.index(max(A))
    A = A[i]
    c = 0.5/np.sqrt(A)
    if   (i == 0):
        q = c*np.array([A, R[2][1]-R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1]])
    elif (i == 1):
        q = c*np.array([R[2][1]-R[1][2], A, R[1][0]+R[0][1], R[0][2]+R[2][0]])
    elif (i == 2):
        q = c*np.array([R[0][2]-R[2][0], R[1][0]+R[0][1], A, R[2][1]+R[1][2]])
    else:
        q = c*np.array([R[1][0]-R[0][1], R[0][2]+R[2][0], R[2][1]+R[1][2], A])
    return q

# Transform Matrices:
def T_from_Rp(R, p):
    return np.vstack((np.hstack((R,p)),
                      np.array([0.0, 0.0, 0.0, 1.0])))

def T_from_origin(origin):
    return T_from_Rp(R_from_rpy(origin.rpy), p_from_xyz(origin.xyz))
def T_from_axisangle(axis, theta):
    return T_from_Rp(R_from_axisangle(axis, theta), np.zeros((3,1)))


#
#   Kinematics Class
#
#   This encapsulates the kinematics functionality, storing the
#   kinematic chain elements.
#
class Kinematics:
    def __init__(self, robot, baseframe, tipframe):
        # Report.
        rospy.loginfo("Kinematics: Setting up the chain from '%s' to '%s'...",
                      baseframe, tipframe)

        # Create the list of joints from the base frame to the tip
        # frame.  Search backwards, as this could be a tree structure.
        self.joints = []
        frame = tipframe
        while (frame != baseframe):
            joint = next((j for j in robot.joints if j.child == frame), None)
            if (joint is None):
                rospy.logerr("Unable find joint connecting to '%s'", frame)
                raise Exception()
            if (joint.parent == frame):
                rospy.logerr("Joint '%s' connects '%s' to itself",
                             joint.name, frame)
                raise Exception()
            self.joints.insert(0, joint)
            frame = joint.parent

        # Report.
        self.dofs = sum(1 for j in self.joints if j.type != 'fixed')
        rospy.loginfo("Kinematics: %d active DOFs, %d total steps",
                      self.dofs, len(self.joints))

    def fkin(self, theta):
        # Check the number of joints
        if (len(theta) != self.dofs):
            rospy.logerr("Number of joint angles (%d) does not match URDF (%d)",
                         len(theta), self.dofs)
            return

        # Initialize the T matrix to walk up the chain, the index to
        # count the moving joints, a list to hold only the continuous joints,
        # and a list to hold the transformation matrices of the continuous joints
        # (from the base frame)
        T     = np.eye(4)
        index = 0
        contjoints = []
        contjointsT = []

        # The transform information is stored in the robot's joint entries.
        for joint in self.joints:
            if (joint.type == 'fixed'):
                T = T @ T_from_origin(joint.origin)
            elif (joint.type == 'continuous'):
                T = T @ T_from_origin(joint.origin) \
                    @ T_from_axisangle(joint.axis, theta[index])
                contjoints.append(joint)
                contjointsT.append(T)
                index += 1
            else:
                rospy.logwarn("Unknown Joint Type: %s", joint.type)

        # COMPUTING A CROSS PRODUCT.  TO MAINTAIN THE COLUMN VECTOR,
        # SPECIFY "axis=0":
        # np.cross(VECTOR1, VECTOR2, axis=0)

        # Compute the Jacobian
        J = np.zeros((6,index))
        jointtip = contjoints[index - 1]
        for i in range(index):
            joint = contjoints[i]
            jointR = R_from_T(contjointsT[i])
            J[0:3,i:i+1] = np.cross(jointR @ e_from_axis(joint.axis), \
                p_from_T(T) - p_from_T(contjointsT[i]), axis=0)
            J[3:6,i:i+1] = jointR @ e_from_axis(joint.axis)

        # Return the Ttip and Jacobian.
        return (T,J)
        
    def ikin(self, xgoal, theta_guess = \
        np.array([0.0, math.pi/2, -math.pi/2]).reshape((3,1))):
        # Newton-Raphson algorithm
        theta = theta_guess
        (T, J) = self.fkin(theta)
        Jinv = np.linalg.inv(J[0:3, 0:3])
        x = p_from_T(T)
        iternum = 0
        
        while np.linalg.norm(xgoal - x) > 0.000001:
            theta = theta + Jinv @ (xgoal - x)
            (T, J) = self.fkin(theta)
            Jinv = np.linalg.inv(J[0:3, 0:3])
            x = p_from_T(T)
            iternum += 1
            if iternum > 100:
                rospy.logerr("Newton-Raphson failed to converge in 100 iterations")
                raise Exception()
        
        # Return theta
        return (theta)


#
#  Main Code - Test (if run independently)
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('kinematics')

    # Grab the robot's URDF from the parameter server.
    robot = Robot.from_parameter_server()

    # Instantiate the Kinematics
    kin = Kinematics(robot, 'world', 'tip')

    # Pick the test angles of the robot.
    theta = [np.pi/4, np.pi/6, np.pi/3]

    # Compute the kinematics.
    (T,J) = kin.fkin(theta)

    # Report.
    np.set_printoptions(precision=6, suppress=True)
    print("Galilea's results")
    print("T:\n", T)
    print("J:\n", J)
    print("p/q: ", p_from_T(T).T, q_from_R(R_from_T(T)))
