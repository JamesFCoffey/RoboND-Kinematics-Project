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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
        s = {alpha0:     0, a0:      0, d1:   0.75, q1: q1,
             alpha1: -pi/2, a1:   0.35, d2:      0, q2: q2-pi/2,
             alpha2:     0, a2:   1.25, d3:      0, q3: q3,
             alpha3: -pi/2, a3: -0.054, d4:   1.50, q4: q4,
             alpha4:  pi/2, a4:      0, d5:      0, q5: q5,
             alpha5: -pi/2, a5:      0, d6:      0, q6: q6,
             alpha6:     0, a6:      0, d7:  0.303, q7: 0}

	# Define Modified DH Transformation matrix
        def DHTmatrix(q, d, a, alpha):
            DHTmatrix = Matrix([[             cos(q),            -sin(q),           0,             a],
                                [ sin(q1)*cos(alpha), cos(q1)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                                [ sin(q1)*sin(alpha), cos(q1)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                                [                  0,                  0,           0,             1]])
            return DHTmatrix

        # Create individual transformation matrices
        T0_1 = DHTmatrix(q1, d1, a0, alpha0).subs(s)
        T1_2 = DHTmatrix(q2, d2, a1, alpha1).subs(s)
        T2_3 = DHTmatrix(q3, d3, a2, alpha2).subs(s)
        T3_4 = DHTmatrix(q4, d4, a3, alpha3).subs(s)
        T4_5 = DHTmatrix(q5, d5, a4, alpha4).subs(s)
        T5_6 = DHTmatrix(q6, d6, a5, alpha5).subs(s)
        T6_7 = DHTmatrix(q7, d7, a6, alpha6).subs(s)

        T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])


            r, p, y = symbols('r p y')

            # Define Roll Matrix about x-axis
            R_x = Matrix([[      1,       0,       0],
                          [      0,  cos(r), -sin(r)],
                          [      0,  sin(r),  cos(r)]])

            # Define Pitch Matrix about y-axis
            R_y = Matrix([[ cos(p),       0,  sin(p)],
                          [      0,       1,       0],
                          [-sin(p),       0,  cos(p)]])

            # Define Yaw Matrix about z-axis
            R_z = Matrix([[ cos(y), -sin(y),       0],
                          [ sin(y),  cos(y),       0],
                          [      0,       0,       1]])

            # Compensate for rotation discrepancy between DH parameters and Gazebo
	    R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2)

            Rrpy = R_x.subs(r, roll) * R_y.subs(p, pitch) * R_z.subs(y, yaw) * R_corr

            # Calculate Wrist Center position
            EE = Matrix([[px], [py], [pz]])

            WC = EE - (0.303) * Rrpy(:,2)

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1],WC[0])

            # Calculate Sides of Triangle Formed by Origin 2, Origin 3, and the WC Origin
            side_a = sqrt(-0.054^2+1.50^2)
            side_b = sqrt(sqrt(WC[0]^2 + WC[1]^2) - 0.35)^2 + (WC[2]-0.75)^2)
            side_c = sqrt(1.25^2+0^2)

            # Calculate Angles of Triangle Formed by Origin 2, Origin 3, and the WC Origin
            angle_a = acos((side_b^2 + side_c^2 - side_a^2) / (2 * side_b * side_c))
            angle_b = acos((side_a^2 + side_c^2 - side_b^2) / (2 * side_a * side_c))
            angle_c = acos((side_a^2 + side_b^2 - side_c^2) / (2 * side_a * side_b))

            
            theta2 = 
            theta3 =
            theta4 =
            theta5 = 
            theta6 = 

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
