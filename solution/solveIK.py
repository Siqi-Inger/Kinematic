#!/usr/bin/env python
import os
import sys
'''
path_ws = os.path.abspath('../../..') 
sys.path.append(path_ws)
sys.path.append(path_ws + '/advance_robotics_assignment/')
sys.path.append(path_ws + '/advance_robotics_assignment/franka_ros_interface')
'''
import numpy as np
from solution.solveFK import FK
from scipy.spatial.transform import Rotation as R

fk = FK()

class IK:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    # fk = FK()

    def __init__(self):
        pass

    @staticmethod
    def calcJacobian(q):
        """
        Calculate the Jacobian of the end effector in a given configuration.
        INPUT:
        q - 1 x 7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUT:
        J - the Jacobian matrix 
        """

        J = []       
        # YOUR CODE STARTS HERE
        # first step: obtain all the transformation matrix
        T_all = []
        Z_all = []
        P_all = []
        # Z_all.append([0, 0, 1])
        # P_all.append([0, 0, 0])
        for i in range(len(q)):
            _, T = fk.forward(q[0:i+1])
            z = T[0:3, 2]
            p = T[0:3, 3]
            T_all.append(T)
            Z_all.append(z)
            P_all.append(p)
        
        Z_all = np.array(Z_all)
        P_all = np.array(P_all)
        
        for i in range(1, len(q)+1):
            Jaco1 = np.cross(Z_all[i-1], (P_all[-1]-P_all[i-1]))
            Jaco2 = Z_all[i-1]
            Jaco = np.append(Jaco1.reshape(-1, 1), Jaco2.reshape(-1, 1), axis=0)
            if i == 1:
                J = Jaco
            else:
                J = np.append(J, Jaco, axis=1)
        # YOUR CODE ENDS HERE
        return J

    @staticmethod
    def cal_target_transform_vec(target, current):
        """
        Calculate the displacement vector and axis of rotation from 
        the current frame to the target frame

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
                 end effector to world

        current - 4x4 numpy array representing the current transformation from
                  end effector to world

        OUTPUTS:
        translate_vec - a 3-element numpy array containing the target translation vector from
                        the current frame to the target frame, expressed in the world frame

        rotate_vec - a 3-element numpy array containing the target rotation vector from
                     the current frame to the end effector frame
        """
        # for the rotate vec, I'm not sure use the rpy(xyz) angle to express or using axis angle to express
        translate_vec = []
        rotate_vec = []
        # YOUR CODE STARTS HERE
        '''
        translate_vec = target[0:3, 3]- current[0:3, 3]

        # convert the rotation matrix into axis-angle
        rotate_target = target[0:3, 0:3]
        rotate_current = current[0:3, 0:3]
        
        # 1. use axis angle
        R_c2t = np.transpose(rotate_current) @ rotate_target
        #use the scipy library
        rotate = R.from_matrix(R_c2t)
        rotate_vec = rotate.as_rotvec()
        # use Rodrigues' formula
        tr = np.trace(R_c2t)

        theta = np.arccos((tr-1)/2)

        rotate_vec = np.array(
            [R_c2t[2, 1] - R_c2t[1, 2],
             R_c2t[0, 2] - R_c2t[2, 0],
             R_c2t[1, 0] - R_c2t[0, 1]]
             ) / (2 * np.sin(theta))
    
        rotate_vec = np.array(rotate_vec)

        rotate_vec = rotate_vec * theta
        '''
        
        # 2. use rpy angle
        position_current = current[:3, 3]
        rotation_current = current[:3, :3]
        R_current = R.from_matrix(rotation_current)
        euler_current = R_current.as_euler('xyz', degrees=True)
        position_target = target[:3, 3]
        rotation_target = target[:3, :3]
        R_target = R.from_matrix(rotation_target)
        euler_target = R_target.as_euler('xyz', degrees=True)
        diff_x = (euler_current[0] - euler_target[0] + np.pi) % (2 * np.pi) - np.pi
        diff_y = (euler_current[1] - euler_target[1] + np.pi) % (2 * np.pi) - np.pi
        diff_z = (euler_current[2] - euler_target[2] + np.pi) % (2 * np.pi) - np.pi
        translate_vec = position_current - position_target
        # diff_x = (euler_target[0] - euler_current[0] + np.pi) % (2 * np.pi) - np.pi
        # diff_y = (euler_target[1] - euler_current[1] + np.pi) % (2 * np.pi) - np.pi
        # diff_z = (euler_target[2] - euler_current[2] + np.pi) % (2 * np.pi) - np.pi
        # translate_vec = position_target - position_current
        rotate_vec = [diff_x, diff_y, diff_z]
        

        ## YOUR CODE ENDS HERE

        return translate_vec, rotate_vec

    def check_joint_constraints(self,q,target):
        """
        Check if the given candidate solution respects the joint limits.

        INPUTS
        q - the given solution (joint angles)

        target - 4x4 numpy array representing the desired transformation from
                 end effector to world

        OUTPUTS:
        success - True if some predefined certain conditions are met. Otherwise False
        """

        success = False
        # YOUR CODE STARTS HERE
        count = 0
        for i in range(len(q)):
            flag = True
            for j in range(len(q[i])):
                if not self.lower[j] <= q[i][j] <= self.upper[j]:
                    flag = False    # the angle doesn't meet the limitation of joints
                    break
            if flag:
                count += 1
        
        if count != 0:
            success = True
        # YOUR CODE ENDS HERE

        return success


    @staticmethod
    def solve_ik(q,target):
        """
        Uses the method you prefer to calculate the joint velocity 

        INPUTS:
        q - the current joint configuration, a "best guess" so far for the final solution

        target - a 4x4 numpy array containing the target end effector pose

        OUTPUTS:
        dq - a desired joint velocity
        Note: Make sure that it will smoothly decay to zero magnitude as the task is achieved.
        """
        dq = []
        # YOUR CODE STARTS HERE
        # compute current and target frams' position and origentation
        _, current = fk.forward(q)
        Jaco = IK.calcJacobian(q)

        pos_error, rot_error = IK.cal_target_transform_vec(target, current)
        error = np.hstack((pos_error, rot_error))

        # 1. Jacobian Transpose Method
        Jaco_t = np.transpose(Jaco)
        dq1 = Jaco_t @ error

        # 2. Pseudoinverse Method
        Jaco_p = np.linalg.pinv(Jaco)
        '''
        J_T = np.transpose(Jaco)
        J_p = J_T @ np.linalg.inv(Jaco @ J_T)
        '''
        dq2 = Jaco_p @ error

        # 3. Damped Least Squares Method
        lam = 0.02
        dq3 = np.linalg.inv(Jaco_t @ Jaco +lam**2 * np.identity(Jaco.shape[1])) @ Jaco_t @ error

        # YOUR CODE ENDS HERE
        return dq2, error

    def inverse(self, target, initial_guess):
        """
        Solve the inverse kinematics of the robot arm

        INPUTS:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world

        initial_guess - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with the solution process (has set up for you)

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.

        success - True if IK is successfully solved. Otherwise False
        """

        q = initial_guess
        success = False

        q_set = []
        error_set = []
        # YOUR CODE STARTS HERE
        max_iter = 100000
        min_error = 0.2
        alpha = 0.5
        # the first time
        for i in range(max_iter):
            dq, error = IK.solve_ik(q, target)
            error_abs = np.abs(error)
            if (np.sum(error_abs)) < min_error:
                q_set.append(q)
                error_set.append(np.sum(error_abs))
            if i % 50000:
                alpha *= 0.5
            q = q + dq * alpha
            # q += dq
        
        success = IK.check_joint_constraints(self, q_set, target)
        print(success)
        # YOUR CODE ENDS HERE

        return q_set, error_set, success

if __name__ == "__main__":
    pass
