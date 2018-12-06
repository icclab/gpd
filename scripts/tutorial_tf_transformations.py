#!/usr/bin/env python  
import roslib
import rospy
import tf
 
if __name__ == '__main__':
     rospy.init_node('my_tf_broadcaster')
     br = tf.TransformBroadcaster()
     rate = rospy.Rate(10.0) # 10 hz
     while not rospy.is_shutdown():
         # sendTransform(translation, rotation, time, child, parent)
         br.sendTransform((2.0, 0.0, 0.0),
                          (0.0, 0.0, 0.0, 1.0),
                          rospy.Time.now(),
                          "robot",
                          "world")
         br.sendTransform((2.0, 0.0, 2.0),
                          (1.0, 0.0, 0.0, 0.0),
                          rospy.Time.now(),
                          "grasp_pose",
                          "robot")
         
#         print("Sent tf transform")
         rate.sleep()
#         print("Time should have passed")
