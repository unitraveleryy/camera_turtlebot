#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
import cv2
import numpy as np


cmd = Twist()

T_OC = (np.load("Desktop/Transformation_mat.npy"))

def cb_Twist(msg:Twist):

    ### Path parameters
    straights = 0.75
    desired_x = 0.55 # 85


    rot = np.array([[msg.angular.x],[msg.angular.y],[msg.angular.z]])
    pos = np.array([[msg.linear.x],[msg.linear.y],[msg.linear.z]])
    rot_mat,_ = cv2.Rodrigues(rot)

    T_CP = np.vstack((np.hstack((rot_mat,pos)),np.array([0,0,0,1])))  # rotation and position of 1 from camera frame

    T_OP = np.matmul(T_OC,T_CP)   # pre multiply with inverse of rot, pos matrix of calibration tag (tag 0), Should give us the rot, pos in tag (0) frame

    rot_OP = T_OP[:3,:3]
    pos_OP = np.array([T_OP[:3,-1]])
    pos_OP = pos_OP.T

    #### this is the angle of the robot (since it is arc cos, itll be only 0 - pi)
    # cos_t = rot_OP[0,0]   #### WRONG must use arcsin
    theta = np.arcsin(rot_OP[1,0])
    x_pos = pos_OP[0,0]
    y_pos = pos_OP[1,0]

    if abs(y_pos) < straights:
        # robot is on straights

        if x_pos > 0:
            #pass # robot is on right side
            err_x = x_pos - desired_x
            cmd.angular.z = - theta + 2*err_x
            
        else:
            #pass # robot is on left side
            err_x = -x_pos - desired_x
            cmd.angular.z = theta + 2*err_x

    else:
        cmd.angular.z = 0
    #     #robot is on curves, THINK RADIALLY
        if y_pos > 0:
            #pass # robot is on the top
            d_robot = np.sqrt(x_pos**2 + (y_pos - straights)**2)
            err_d = d_robot - desired_x
            if x_pos > 0:
                phi = np.arcsin((y_pos - straights)/d_robot)
            else:
                phi = np.pi - np.arcsin((y_pos - straights)/d_robot)
                theta = np.pi - theta
            
            cmd.angular.z = - (theta - phi) + 2*err_d



        else:
            #pass # robot is on the top
            d_robot = np.sqrt(x_pos**2 + (y_pos + straights)**2)
            err_d = d_robot - desired_x
            if x_pos > 0:
                phi = 2*np.pi + np.arcsin((y_pos + straights)/d_robot)
                theta = 2*np.pi + theta
            else:
                phi = np.pi - np.arcsin((y_pos + straights)/d_robot)
                theta = np.pi - theta
            a = - (theta - phi)
            # if a < 0:
            #     a += 2*np.pi
            cmd.angular.z = a + 2*err_d
            

    print(cmd.angular.z*360/(2*np.pi))
    # print(cmd.angular.z)
    # cmd.angular.z = 0

    #cmd.linear.x = .10
    pub.publish(cmd)
    #print(rot_OP)
    #print(pos_OP)
    #print(T_OP)




if __name__=="__main__":
    rospy.init_node("testing_fields")
    rospy.loginfo("Node is Live")

    sub_Twist_1 = rospy.Subscriber("/april_data_Twist1",Twist,callback=cb_Twist)
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()