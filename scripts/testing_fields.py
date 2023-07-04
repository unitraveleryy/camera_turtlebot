#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
import cv2
import numpy as np

# tfm_mat1 = (np.load("Desktop/Calibtarion_mtx.npy"))
# tfm_mat = np.linalg.inv(tfm_mat1)
# shifted_origin_c = np.array(np.load("Desktop/Calibration_point.npy"))

# shifted_origin_f = np.matmul(tfm_mat, shifted_origin_c) # in cooridnate frame f


# #print(transformation_matrix)
# #print(np.matmul(tfm_mat1,tfm_mat))


# def cb_Twist(msg:Twist):
#     rot = np.array([[msg.angular.x],[msg.angular.y],[msg.angular.z]])
#     rot_mat,_ = cv2.Rodrigues(rot) #this is rotation matrix of the tag in camera coordinates
#     rot_mat_f = np.matmul(tfm_mat,rot_mat)  # rotation matrix in the coordinate frame of the calibration tag
#     print("new mtx\n",rot_mat_f,"\n")

#     position_c = np.matmul(tfm_mat, np.array([[msg.linear.x],[msg.linear.y],[msg.linear.z]]))
#     position_f = position_c - shifted_origin_f  # position wrt the new origin 
#     #print(position_f)




# if __name__ == "__main__":
#     rospy.init_node("testing_fields")
#     rospy.loginfo("node is live")

#     sub_Twist_0 = rospy.Subscribe8.885r("/april_data_Twist0",Twist,callback=cb_Twist)

#     while not rospy.is_shutdown():
#         rospy.spin()



# Loading Transformation matrix





T_OC = (np.load("Desktop/Transformation_mat.npy"))
rot_OP1 = (np.load("Desktop/Calibtarion_mtx.npy"))
pos_OP1 = np.array(np.load("Desktop/Calibration_point.npy"))

def cb_Twist(msg:Twist):
    rot = np.array([[msg.angular.x],[msg.angular.y],[msg.angular.z]])
    pos = np.array([[msg.linear.x],[msg.linear.y],[msg.linear.z]])
    rot_mat,_ = cv2.Rodrigues(rot)

    T_CP = np.vstack((np.hstack((rot_mat,pos)),np.array([0,0,0,1])))  # rotation and position of 1 from camera frame

    T_OP = np.matmul(T_OC,T_CP)   # pre multiply with inverse of rot, pos matrix of calibration tag (tag 0), Should give us the rot, pos in tag (0) frame

    rot_OP = T_OP[:3,:3]
    pos_OP = np.array([T_OP[:3,-1]])
    pos_OP = pos_OP.T

    print(rot_OP)
    print(pos_OP)
    print(T_OP)




if __name__=="__main__":
    rospy.init_node("testing_fields")
    rospy.loginfo("Node is Live")

    sub_Twist_1 = rospy.Subscriber("/april_data_Twist1",Twist,callback=cb_Twist)

    while not rospy.is_shutdown():
        rospy.spin()