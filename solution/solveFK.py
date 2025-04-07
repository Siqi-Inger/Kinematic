#!/usr/bin/env python
import math
import numpy as np
from math import pi, cos, sin

class FK():

    def __init__(self):
        # Define geometric parameters for computing the forward kinematics. 
        # The required parameters are provided in the assignment description document.
        self.dh_params = self.init_dh_params()
        self.joint_offsets = self.init_joint_offsets()

    def init_dh_params(self):
        """
        Initialize dh parameters from all intermediate frames in the form [a, alpha, d]
        (refer to assignment description)
        """
        a =[0, 0, 0.082, -0.082, 0, 0.088, 0]
        alpha = [-pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2, 0]
        d = [0.333, 0, 0.316, 0, 0.384, 0, 0.21]

        dh_params = np.c_[a, alpha]
        dh_params = np.c_[dh_params, d]
        return dh_params

    def init_joint_offsets(self):
        """
        Initialize joint position offsets
        relative to intermediate frames defined using
        DH conventions 
        (refer to assignment description)
        """
        x =[0, 0, 0, 0, 0, 0, 0]
        y = [0, 0, 0, 0, 0, 0, 0]
        z = [0.141, 0, 0.195, 0, 0.125, 0, 0.051]

        joint_offsets = np.c_[x, y]
        joint_offsets = np.c_[joint_offsets, z]
        return joint_offsets

    def build_dh_transform(self, a, alpha, d, theta):
        """
        Construct transformation matrix T,
        using DH parameters and conventions
        """
        
        T = []
        # YOUR CODE STARTS HERE
        T = [
            np.array([cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)]),
            np.array([sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)]),
            np.array([0, sin(alpha), cos(alpha), d]),
            np.array([0, 0, 0, 1])
        ]
    
        # YOUR CODE ENDS HERE
        return T

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                         Each row contains the [x,y,z] coordinates in the world frame of the respective 
                         joint's center in meters. The base of the robot is located at [0,0,0].

        T0e - a homogeneous transformation matrix,
              representing the end effector frame expressed in the world frame
        """

        jointPositions = []
        T0e = []
        # YOUR CODE STARTS HERE
        T_previous = np.identity(4)
        if len(q) > 7:
            length = 7
        else:
            length = len(q)
        for i in range(length):
            a, alpha, d = self.dh_params[i]
            T = self.build_dh_transform(a, alpha, d, q[i])
            T = T_previous @ T
            #  be careful about @, *, np.dot operate.
            # x, y, z = self.joint_offsets[i]
            joint_offset = np.insert(self.joint_offsets[i], 3, values=1, axis=0)
            position = T @ joint_offset
            # position = np.delete(position, 3, axis=0)
            jointPositions.append(position[:3])
            T_previous = T
        T0e = T_previous
        jointPositions = np.array(jointPositions)
    
        # YOUR CODE ENDS HERE
        return jointPositions, T0e

if __name__ == "__main__":
    pass
