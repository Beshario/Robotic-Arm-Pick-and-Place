#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Beshari Jamal

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
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
   	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	#   
	# Create Modified DH parameters
	s = {alpha0:       0, a0:      0, d1:  0.75, q1:      q1,
    	    alpha1:    -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
     	    alpha2:        0, a2:   1.25, d3:     0, q3:	  q3,
     	    alpha3:    -pi/2, a3: -0.054, d4:   1.5, q4:	  q1,
     	    alpha4:     pi/2, a4:      0, d5:     0, q5:	  q5,
     	    alpha5:    -pi/2, a5:      0, d6:     0, q6:	  q6,
     	    alpha6:        0, a6:      0, d7: 0.303, q7:      0}

	#            
	# Define Modified DH Transformation matrix
	def Rot_x(q):
            Rx = Matrix([[ 1,      0,       0],
                      [ 0, cos(q), -sin(q)],
                      [ 0, sin(q),  cos(q)]])
            return Rx
    
    
    	def Rot_y(q):
            Ry = Matrix([[  cos(q), 0, sin(q)],
                      [       0, 1,      0],
                      [ -sin(q), 0, cos(q)]])
            return Ry
    
    
    	def Rot_z(q):
            Rz = Matrix([[ cos(q), -sin(q), 0],
                      [ sin(q),  cos(q), 0],
                      [      0,       0, 1]])
            return Rz

    	def TF_Matrix(alpha, a, d, q):
	    TF = Matrix([[             cos(q),            -sin(q),            0,              a],
	 	[ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
		[ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
		[                   0,                   0,            0,               1]])
    	    return TF
	# Conversion factors between radians and degrees
    	rtd = 180 / pi
    	dtr = pi / 180
	#
	# Create individual transformation matrices
	 # Create individual transformation matrices, for now
    	T0_1=TF_Matrix(alpha0, a0, d1, q1).subs(s)
    	T1_2=TF_Matrix(alpha1, a1, d2, q2).subs(s)
    	T2_3=TF_Matrix(alpha2, a2, d3, q3).subs(s)
    	T3_4=TF_Matrix(alpha3, a3, d4, q4).subs(s)
    	T4_5=TF_Matrix(alpha4, a4, d5, q5).subs(s)
    	T5_6=TF_Matrix(alpha5, a5, d6, q6).subs(s)
    	T6_G=TF_Matrix(alpha6, a6, d7, q7).subs(s)

    	T0_G=T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
	R0_3 = T0_1[0:3, 0:3]*T1_2[0:3, 0:3]*T2_3[0:3, 0:3]
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

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
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    r, p, y = symbols('r p y')
   	    #Solution is in DH Reference freme, to convert it to URDF file  reference frame we rortate 180 deg around Z and -90 around Y
	    Rot_corr=Rot_z(180*dtr)*Rot_y(-90*dtr)
    	    R_EE = Rot_z(yaw)*Rot_y(pitch)*Rot_x(roll)*Rot_corr

   	    EE=Matrix([[px],[py],[pz]]) ##EE point location

    	    ##WC is only a translation difference from J EE_to J6 along, the Z axis of the local reference frame
    	    #wrist center
    	    WC= EE - (0.303)*R_EE[:,2]  
	    #
	    # Calculate joint angles using Geometric IK method
	    theta1=atan2(WC[1],WC[0])
    	    side_a=1.50
    	    side_b_xy=sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35 
    	    side_b_z=WC[2]-0.75
   	    side_b=sqrt(pow((side_b_xy),2) + pow((side_b_z), 2))
   	    side_c=1.25
    	    #Angles using cosine laws
    	    angle_a=acos((   side_b * side_b + side_c * side_c - side_a * side_a )/( 2 * side_b * side_c ))
    	    angle_b=acos(( - side_b * side_b + side_c * side_c + side_a * side_a )/( 2 * side_a * side_c ))
    	    angle_c=acos((   side_b * side_b - side_c * side_c + side_a * side_a )/( 2 * side_a * side_b ))
    	    ### Thetas
    	    theta2= pi/2 - angle_a - atan2(side_b_z, side_b_xy)
    	    theta3= pi/2 - angle_b + 0.036 #sag in link 4
    	    #multiplying the the R in T trans     matrieces. (picking the first 3 rows and 3 columns)
    	    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    	    R3_6 = R0_3.T * R_EE
  

    	    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
    	    theta5 = atan2(sqrt(R3_6[0, 2]**2+R3_6[2, 2]**2), R3_6[1, 2])
    	    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

    	    # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    #
            ###
		
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

