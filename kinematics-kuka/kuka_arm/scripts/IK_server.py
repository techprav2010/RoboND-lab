#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

from DHmodel import *

# Model definition
dh_model = None

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here

        # Create symbols
        global dh_model
        dh_model.init_variables()

        # Create Modified DH parameters
        dh_model.init_dh_parameters()

        # Define Modified DH Transformation matrix
        # Create individual transformation matrices
        dh_model.init_transform_matrices()

        # Extract rotation matrices from the transformation matrices


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px, py, pz = get_position(req.poses[x])

            (roll, pitch, yaw) = get_euler(req.poses[x])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    # Calculate joint angles using Geometric IK method
	    #
            ###
            ROT_EE = dh_model.get_rot_end_effector()
            # More information can be found in KR210 Forward Kinematics section

            Rot_Error = dh_model.get_rot_error(radians(180), radians(-90))

            ROT_EE = ROT_EE * Rot_Error
            ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

            EE = Matrix([
                [px],
                [py],
                [pz]
                ])

            WC = EE - (0.303) * ROT_EE[:,2]

            # Calculate joint angles using Geometric IK method
            # More information can be found in the Inverse Kinematics with Kuka KR210
            theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta 2 and theta3
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35),2) + pow((WC[2]-0.75),2))
            side_c = 1.25

            angle_a = acos((side_b*side_b+side_c*side_c-side_a*side_a)/(2*side_b*side_c))
            angle_b = acos((side_a*side_a+side_c*side_c-side_b*side_b)/(2*side_a*side_c))
            angle_c = acos((side_a*side_a+side_b*side_b-side_c*side_c)/(2*side_a*side_b))

            theta2 = pi / 2 - angle_a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
            theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m

            R0_3 = dh_model.T0_1[0:3,0:3] * dh_model.T1_2[0:3,0:3] * dh_model.T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={dh_model.q1:theta1, dh_model.q2:theta2, dh_model.q3:theta3})

            R3_6 = R0_3.transpose() * ROT_EE

            # Euler angles from rotation matrix
            # More information can be found in the Euler Angles from a Rotation Matrix section

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            a = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])
            theta5 = atan2(a, R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    global dh_model

    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')

    # Internal model initialization
    dh_model = DHmodel()

    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
