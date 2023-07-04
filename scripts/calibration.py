#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Quaternion, Point
import cv2
import numpy as np


itera = 25

class Rotation:
    def __init__(self) -> None:
        self.rot_col = np.zeros((3,itera))
        #self.rot_idx = 0
        self.pos_col = np.zeros((3,itera))
        #self.pos_idx = 0
        self.idx = 0



# def cb_rot(msg:Point):
#     if inst.rot_idx < itera:
#         inst.rot_col[0][inst.rot_idx] = msg.x
#         inst.rot_col[1][inst.rot_idx] = msg.y
#         inst.rot_col[2][inst.rot_idx] = msg.z
#         inst.rot_idx = inst.rot_idx + 1
#     print("rot iteration",inst.rot_idx)


# def cb_pos(msg:Point):
#     if inst.pos_idx < itera:
#         inst.pos_col[0][inst.pos_idx] = msg.x
#         inst.pos_col[1][inst.pos_idx] = msg.y
#         inst.pos_col[2][inst.pos_idx] = msg.z
#         inst.pos_idx = inst.pos_idx + 1
#     if inst.pos_idx == itera and inst.rot_idx == itera:
#         rospy.signal_shutdown("Done")
#     print("pos iteration",inst.pos_idx)

def cb_Twist(msg:Twist):
    if inst.idx < itera:
        inst.pos_col[0][inst.idx] = msg.linear.x
        inst.pos_col[1][inst.idx] = msg.linear.y
        inst.pos_col[2][inst.idx] = msg.linear.z

        inst.rot_col[0][inst.idx] = msg.angular.x
        inst.rot_col[1][inst.idx] = msg.angular.y
        inst.rot_col[2][inst.idx] = msg.angular.z

        inst.idx = inst.idx + 1
    print("iteration",inst.idx)
    if inst.idx == itera:
        rospy.signal_shutdown("Done")
    


def calculate():

    rot = np.mean(inst.rot_col,axis=1)
    scaling_factor = 68.4/12 # (size of aruco tag 8 cm)8.885
    pos = np.array([scaling_factor*np.mean(inst.pos_col,axis=1)])
    pos = pos.T
    matrix,_ = cv2.Rodrigues(rot)
    mat = np.vstack((np.hstack((matrix,pos)),np.array([0,0,0,1])))
    mat_inv = np.vstack((np.hstack((matrix.T,np.matmul(-matrix.T,pos))),np.array([0,0,0,1])))
    
    # np.save("Desktop/Calibtarion_mtx.npy", matrix)
    # np.save("Desktop/Calibration_point.npy",pos)
    np.save("Desktop/Transformation_mat.npy",mat_inv)
    
    print(matrix)
    print(pos)
    print(mat_inv)


if __name__ == "__main__":
    rospy.init_node("calbration")
    rospy.loginfo("node is live")
    rospy.on_shutdown(calculate)

    inst = Rotation()
    
    sub_0 = rospy.Subscriber("/april_data_Twist0",Twist,callback=cb_Twist)
    #sub_pos_0 = rospy.Subscriber("/april_data_pos0",Point,callback=cb_pos)

    while not rospy.is_shutdown():
        rospy.spin()

