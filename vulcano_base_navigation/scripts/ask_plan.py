#!/usr/bin/env python

import sys
import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

def ask_plan(start, goal, tolerance):
    service = '/move_base/GlobalPlanner/make_plan'
    rospy.wait_for_service(service)
    try:
        get_plan = rospy.ServiceProxy(service, GetPlan)
        response = get_plan(start, goal)
        return response.plan

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) > 1:
        print usage()
        sys.exit(1)

    rospy.init_node("ask_plan")
    start = PoseStamped()
    start.header.frame_id = '/odom'
    start.header.stamp = rospy.Time.now()
    start.pose.position.x = 0
    start.pose.position.y = 0
    start.pose.position.z = 0
    start.pose.orientation.x = 0
    start.pose.orientation.y = 0
    start.pose.orientation.z = 0
    start.pose.orientation.w = 1

