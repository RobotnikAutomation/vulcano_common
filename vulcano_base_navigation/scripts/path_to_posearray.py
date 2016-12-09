#!/usr/bin/env python

import sys
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray



class PathToPoseArray(object):
    def __init__(self):

        self.posearray_publisher = rospy.Publisher("~plan_pose", PoseArray, queue_size=1)
        rospy.Subscriber("plan", Path, self.path_callback)

    def path_callback(self, path):
        posearray = PoseArray()
        posearray.header = path.header
        for pose_stamped in path.poses:
            posearray.poses.append(pose_stamped.pose)
        self.posearray_publisher.publish(posearray)


if __name__ == "__main__":
    rospy.init_node("path_to_posearray")

    p2p = PathToPoseArray()
    rospy.spin()

