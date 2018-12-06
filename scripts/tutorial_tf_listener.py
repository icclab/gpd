#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from tf.transformations import *
from math import pi

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/grasp_pose', rospy.Time(0))
            print("rotation quaternion world->grasp_pose (x,y,z,w): " + str(rot))
            orig = rot
            q_rot = quaternion_from_euler(0, pi/2, 0) # rotate 90 degrees around y axis
            print("rotation quaternion +90 degrees around y (x,y,z,w): " + str(q_rot))
            
            rotated_pose = quaternion_multiply(orig, q_rot) 
            print("rotated pose (x,y,z,w): " + str(rotated_pose))
            #sendTransform(translation, rotation, time, child, parent)
            
            br.sendTransform((0, 0, 0.5),
                          (float(q_rot[0]), float(q_rot[1]), float(q_rot[2]), float(q_rot[3])),
                          rospy.Time.now(),
                          "directly_rotated_grasp_pose",
                          "grasp_pose")
            trans[2] += 0.5
            br.sendTransform(trans,
                          (float(rotated_pose[0]), float(rotated_pose[1]), float(rotated_pose[2]), float(rotated_pose[3])),
                          rospy.Time.now(),
                          "rotated_grasp_pose",
                          "world")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
         #listener.waitForTransform('/world',"/grasp_pose", rospy.Time(), rospy.Duration(10.0))
         

        rate.sleep()
