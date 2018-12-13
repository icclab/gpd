#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Trigger

def call_pointcloud_filter_service():
    rospy.wait_for_service('filter_pointcloud')
    try:
        service_proxy = rospy.ServiceProxy('filter_pointcloud', Trigger)
        resp1 = service_proxy()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print "Requesting point cloud filtering"
    call_pointcloud_filter_service()
