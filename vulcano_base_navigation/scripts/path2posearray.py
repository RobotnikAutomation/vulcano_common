#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Path

class Path2PoseArray(object):
    def __init__(self):
        self.value = 0

        rospy.init_node('path2posearray')

        self.pub = rospy.Publisher('plan_posearray', PoseArray, queue_size = 1)
        rospy.Subscriber('plan', Path, self.callback)

    def callback(self, path):
        posearray = PoseArray()
        posearray.header = path.header
        for posestamped in path.poses:
            posearray.poses.append(posestamped.pose)
        self.pub.publish(posearray)

if __name__ == '__main__':

    Path2PoseArray()
    rospy.spin()
