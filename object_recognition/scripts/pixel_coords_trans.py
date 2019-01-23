#!/usr/bin/env python
# coding: UTF-8
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped

# cx = 319.5
# cy = 239.5
# fx = 570.3422241210938
# fy = 570.3422241210938

cx = 31.577287
cy = 229.497412
fx = 540.12097
fy = 540.12097


#for realsense d435
# cx = 636.2023315429688
# cy = 371.5960388183594
# fx = 918.495849609375
# fy = 918.3920288085938

def camera_pixel_trans(x, y, z):
    x_camera = z*(cx-x)/fx
    y_camera = z*(cy-y)/fy
    return x_camera, y_camera

def callback(obj_pose):
    # object position and orientarion on pixel coordinates
    x = obj_pose.pose.position.x
    y = obj_pose.pose.position.y
    z = obj_pose.pose.position.z
    # z = 0.63
    qx = obj_pose.pose.orientation.x
    qy = obj_pose.pose.orientation.y
    qz = obj_pose.pose.orientation.z
    qw = obj_pose.pose.orientation.w
    print "----------------"
    print x, y, z

    #object position on camera coordinates
    x_camera, y_camera = camera_pixel_trans(x, y, z)
    print x_camera, y_camera

    br.sendTransform((z, x_camera, y_camera),
                     (qx, qy, qz, qw),
                     rospy.Time.now(),
                     "object",
                     # "camera_link")
                     "camera_rgb_frame")
    rospy.timer.sleep(0.001)

if __name__ == '__main__':
    rospy.init_node('camera_obj_tf')
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.Subscriber("/linemod/obj_pose", PoseStamped, callback)
    rospy.spin()
