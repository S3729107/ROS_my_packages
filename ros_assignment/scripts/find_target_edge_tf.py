#! /usr/bin/env python

import math
import rospy
import message_filters
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import waypoint


DIST_DIFF_OFFSET = 0.5


def getTargetPos(laser_msg):

    curr_diff_distance = 0
    corner_indices = []
    corner_isleft = []
    distances = laser_msg.ranges
    print("Total length of distance list: " + str(len(distances)))

    for i, range in enumerate(distances):
        # avoid null error
        if (i < len(distances) - 1):
            # difference
            curr_diff_distance = distances[i+1] - distances[i]

            if (abs(curr_diff_distance) > DIST_DIFF_OFFSET):
                # corner is on left side if next pos is further away
                is_leftside_corner = (curr_diff_distance > 0)
                corner_isleft.append(is_leftside_corner)

                if (is_leftside_corner):
                    corner_indices.append(i)
                else:
                    corner_indices.append(i + 1)

    print(corner_indices)

    delay = 0.5

    # if a corner is found
    if (len(corner_indices) > 0):
        # will it use the first corner from left side scan (= 0) etc.
        chosen_corner_index = 0

        dest = '/map_frame'
        src = '/base_link'

        listener = tf.TransformListener()
        pose = geometry_msgs.msg.Pose()

        pstamped = geometry_msgs.msg.PoseStamped()
        pstamped.header.stamp = rospy.Time.now() - rospy.Duration(delay)
        pstamped.header.frame_id = src
        pstamped.pose = pose

        transposed = listener.transformPose(dest, pstamped)

        print(transposed)

    # no corner found
    else:
        print("No corner found")


rospy.init_node('find_target_pos_edge')

#dom_sub = message_filters.Subscriber('/odom', Odometry)
#laser_sub = message_filters.Subscriber('/scan', LaserScan)

#ts = message_filters.TimeSynchronizer([odom_sub, laser_sub], 10)
# ts.registerCallback(getTargetPos)

#tf_sub = rospy.Subscriber('/tf', TFMessage, getTargetPos)
laser_sub = rospy.Subscriber('/scan', LaserScan, getTargetPos)

# print(laser_sub)


rospy.spin()
