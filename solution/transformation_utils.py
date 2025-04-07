#!/usr/bin/env python
import numpy as np
import math
from math import cos, sin

# Note: Complete the following subfunctions to generate valid transformation matrices 
# from a translation vector and Euler angles, or a sequence of 
# successive rotations around z, y, and x.
class transformation():

    @staticmethod
    def trans(d):
        """
        Calculate pure translation homogenous transformation by d
        """

        # YOUR CODE STARTS HERE
        T = [
            np.array([1, 0, 0, d[0]]),
            np.array([0, 1, 0, d[1]]),
            np.array([0, 0, 1, d[2]]),
            np.array([0, 0, 0, 1])
        ]

        return T
    
        # YOUR CODE ENDS HERE
    
    @staticmethod
    def roll(a):
        """
        Calculate homogenous transformation for rotation around x axis by angle a
        """

        # YOUR CODE STARTS HERE
        T = [
            np.array([1, 0, 0, 0]),
            np.array([0, cos(a), -sin(a), 0]),
            np.array([0, sin(a), cos(a), 0]),
            np.array([0, 0, 0, 1])
        ]

        return T
    
        # YOUR CODE ENDS HERE

    @staticmethod
    def pitch(a):
        """
        Calculate homogenous transformation for rotation around y axis by angle a
        """

        # YOUR CODE STARTS HERE
        T = [
            np.array([cos(a), 0, sin(a), 0]),
            np.array([0, 1, 0, 0]),
            np.array([-sin(a), 0, cos(a), 0]),
            np.array([0, 0, 0, 1])
        ]

        return T
    
        # YOUR CODE ENDS HERE

    @staticmethod
    def yaw(a):
        """
        Calculate homogenous transformation for rotation around z axis by angle a
        """

        # YOUR CODE STARTS HERE
        T = [
            np.array([cos(a), -sin(a), 0, 0]),
            np.array([sin(a), cos(a), 0, 0]),
            np.array([0, 0, 1, 0]),
            np.array([0, 0, 0,1])
        ]

        return T
    
        # YOUR CODE ENDS HERE

    @staticmethod
    def transform(d,rpy):
        """
        Calculate a homogenous transformation for translation by d and
        rotation corresponding to roll-pitch-yaw euler angles
        """

        # YOUR CODE STARTS HERE
        rotate_matrix = np.array(transformation.trans(d))
        rotate_matrix = np.array(rotate_matrix) @ np.array(transformation.roll(rpy[0]))
        rotate_matrix = np.array(rotate_matrix) @ np.array(transformation.pitch(rpy[1]))
        rotate_matrix = np.array(rotate_matrix) @ np.array(transformation.yaw(rpy[2]))
        '''
        rotate_matrix = [
            [cos(roll_angle)*cos(pitch_angle), -sin(roll_angle)*cos(yam_angle) + cos(roll_angle)*sin(pitch_angle)*sin(yam_angle), sin(roll_angle)*sin(yam_angle) + cos(roll_angle)*sin(pitch_angle)*cos(yam_angle)],
            [sin(roll_angle)*cos(pitch_angle), cos(roll_angle)*cos(yam_angle) + sin(roll_angle)*sin(pitch_angle)*sin(yam_angle), -cos(roll_angle)*sin(yam_angle) + sin(roll_angle)*sin(pitch_angle)*cos(yam_angle)],
            [-sin(pitch_angle), cos(pitch_angle)*sin(yam_angle), cos(pitch_angle)*cos(yam_angle)]
        ]
        homegenous_matrix = np.insert(rotate_matrix, 3, values=d, axis=1)
        homegenous_matrix = np.insert(homegenous_matrix, 3, values=[0, 0, 0, 1], axis=0)
        '''

        return rotate_matrix
    
        # YOUR CODE ENDS HERE
    
if __name__ == "__main__":
    pass
