#!/usr/bin/env python
# coding: UTF-8
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from abe_manipulator_h.srv import get_param
from abe_manipulator_h.srv import get_paramResponse

trans0 = PoseStamped()
trans1 = PoseStamped()


#2018_7_11取得 xtion カメラ内部パラメータ
cx = 311.577287
cy = 229.497412
fx = 540.12097
fy = 540.12097


def camera_pixel_trans(x, y, z):
    x_camera = z*(cx-x)/fx
    y_camera = z*(cy-y)/fy
    return x_camera, y_camera


def callback(obj_pose):
    object_name = int(obj_pose.header.frame_id)
    flame_name = "object" + str(object_name)

    global trans0
    global trans1
    
    x = obj_pose.pose.position.x
    y = obj_pose.pose.position.y
    z = obj_pose.pose.position.z
    qx = obj_pose.pose.orientation.x
    qy = obj_pose.pose.orientation.y
    qz = obj_pose.pose.orientation.z
    qw = obj_pose.pose.orientation.w
    print  "---------------------------------"
    print ("object name:" ,object_name)
    print x, y, z
    
    #object position on camera coordinates
    x_camera, y_camera = camera_pixel_trans(x, y, z)
    print x_camera, y_camera
    
    br.sendTransform((z, x_camera, y_camera),(qx, qy, qz, qw), rospy.Time.now(), flame_name , "camera_rgb_frame")
    # br.sendTransform((z, x_camera, y_camera),(0,0,0,1), rospy.Time.now(), flame_name , "camera_rgb_frame")
    
    rospy.timer.sleep(0.001)
    trans_ = listener.lookupTransform("world",flame_name,rospy.Time(0))
    
    if object_name == 0:
        trans0.pose.position.x = trans_[0][0]
        trans0.pose.position.y = trans_[0][1]
        trans0.pose.position.z = trans_[0][2]
        trans0.pose.orientation.x = trans_[1][0]
        trans0.pose.orientation.y = trans_[1][1]
        trans0.pose.orientation.z = trans_[1][2]
        trans0.pose.orientation.w = trans_[1][3]
        print trans0
    else :
        trans1.pose.position.x = trans_[0][0]
        trans1.pose.position.y = trans_[0][1]
        trans1.pose.position.z = trans_[0][2]
        trans1.pose.orientation.x = trans_[1][0]
        trans1.pose.orientation.y = trans_[1][1]
        trans1.pose.orientation.z = trans_[1][2]
        trans1.pose.orientation.w = trans_[1][3]
        print trans1



def param_server():
    rospy.init_node('camera_obj_tf',anonymous = True)
    ser = rospy.Service('/getObjectParam',get_param,param_update)

def param_update(obj_num):

    if obj_num.object_num == 0:
        x = trans0.pose.position.x
        y = trans0.pose.position.y
        z = trans0.pose.position.z
        roll  = trans0.pose.orientation.x
        pitch = trans0.pose.orientation.y
        yaw   = trans0.pose.orientation.z
        return get_paramResponse(x, y, z, roll, pitch, yaw)  
        
        
    else :
        x = trans1.pose.position.x
        y = trans1.pose.position.y
        z = trans1.pose.position.z
        roll  = trans1.pose.orientation.x
        pitch = trans1.pose.orientation.y
        yaw   = trans1.pose.orientation.z        
        return get_paramResponse(x, y, z, roll, pitch, yaw)   
        

if __name__ == '__main__':
    param_server()
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.Subscriber("/linemod/obj_pose", PoseStamped, callback)        #xyz rpy q の情報を購読
    r = rospy.Rate(10)
    rospy.spin()
