#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan

DIST_DIFF_OFFSET = 0.5


def getTargetPos(msg):

    curr_diff_distance = 0
    corner_indices = []
    corner_isleft = []
    distances = msg.ranges
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

    # if a corner is found
    if (len(corner_indices) > 0):
        # will it use the first corner from left side scan (= 0) etc.
        chosen_corner_index = 0

        # get pos, angle of robot (get transform)
        curr_pos_x = 0
        curr_pos_y = 0
        curr_angle = 0

        robot_width = 0

        # calculate x, y vector of corner
        corner_index = corner_indices[chosen_corner_index]
        # //distance of corner point
        corner_dist = distances[corner_index]
        # //convert from 720 laser list into the 270 degrees and get angle
        relative_corner_angle = (corner_index / 720) * 270
        cartesian_corner_angle = relative_corner_angle - curr_angle  # FIX THIS ONE
        # //convert to radians
        corner_angle_radians = cartesian_corner_angle * (math.pi / 180)
        # //get x, y from dist, dir, and current pos
        corner_x = curr_pos_x + (corner_dist * math.cos(corner_angle_radians))
        corner_y = curr_pos_y + (corner_dist * math.sin(corner_angle_radians))

        # get isleft or right corner
        isleft = corner_isleft[chosen_corner_index]

        # get offset from corner side (((CHECK THE SIGNS))) (((offset amount to be decided)))
        corner_x_offset = 0
        corner_y_offset = 0
        if (isleft):
            corner_x_offset = robot_width
            corner_y_offset = -robot_width
        else:
            corner_x_offset = -robot_width
            corner_y_offset = robot_width

        # calc next pos from corner + offset
        target_x = corner_x + corner_x_offset
        target_y = corner_y + corner_y_offset

        print("Target x = " + str(target_x))
        print("Target y = " + str(target_y))

        # THEN PUBLISH TARGET TO NEW MESSAGE TO BE READ BY OTHER

    # no corner found
    else:
        print("No corner found")


rospy.init_node('find_target_pos_edge')
sub = rospy.Subscriber('/scan', LaserScan, getTargetPos)

rospy.spin()
