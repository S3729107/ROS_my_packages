#! /usr/bin/env python

import math
import rospy
import message_filters
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import waypoint


DIST_DIFF_OFFSET = 0.5


def getTargetPos(odom_msg, laser_msg):

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

    # if a corner is found
    if (len(corner_indices) > 0):
        # will it use the first corner from left side scan (= 0) etc.
        chosen_corner_index = 0

        #listener = tf.TransformListener()

        # get pos, angle of robot (get transform)
        odom = odom_msg.pose.pose
        pos = odom.position

        curr_pos_x = pos.x
        curr_pos_y = pos.y
        print("Current x = " + str(curr_pos_x))
        print("Current y = " + str(curr_pos_y))

        orientation = odom.orientation
        orientation_list = [orientation.x,
                            orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        yaw = (yaw * 180 / math.pi)
        yaw = yaw % 360
        print("Current angle = " + str(yaw))

        # cartesian angle
        curr_angle = yaw + 90
        curr_angle = -curr_angle % 360
        print("Current cart angle = " + str(curr_angle))

        corner_offset = 0  # account for robot width etc

        # # calculate x, y vector of corner
        # corner_index = corner_indices[chosen_corner_index]
        # # //distance of corner point
        # corner_dist = distances[corner_index]
        # # //convert from 720(or 1974) laser list into the 270 degrees and get angle
        # relative_corner_angle = (corner_index / 720) * 270  # sim
        # # ///// relative_corner_angle = (corner_index / len(distances)) * 360 # irl
        # relative_corner_angle = (-relative_corner_angle + 135) % 360
        # print("Target relative angle = " + str(relative_corner_angle))
        # cartesian_corner_angle = (
        #     curr_angle + relative_corner_angle) % 360  # FIX THIS ONE
        # print("Target cart angle = " + str(cartesian_corner_angle))
        # # //convert to radians
        # corner_angle_radians = cartesian_corner_angle * (math.pi / 180)
        # # //get x, y from dist, dir, and current pos
        # corner_x = curr_pos_x + (corner_dist * math.cos(corner_angle_radians))
        # corner_y = curr_pos_y + (corner_dist * math.sin(corner_angle_radians))

        # calculate x, y vector of corner
        corner_index = corner_indices[chosen_corner_index]
        # distance of corner point
        corner_dist = distances[corner_index]
        # convert from 720 laser list to the 270 degrees
        relative_corner_angle = (corner_index / 720) * 270
        # convert to cartesian angle
        relative_corner_angle -= 135  # bring to cart 0
        relative_corner_angle = -relative_corner_angle % 360
        print("Target relative angle = " + str(relative_corner_angle))
        # get world angle of edge
        world_corner_angle = (curr_angle + relative_corner_angle) % 360
        print("Target cart angle = " + str(world_corner_angle))
        # //convert to radians
        corner_angle_radians = world_corner_angle * (math.pi / 180)

        # //get x, y from dist, dir, and current pos
        corner_x = curr_pos_x + (corner_dist * math.cos(corner_angle_radians))
        corner_y = curr_pos_y + (corner_dist * math.sin(corner_angle_radians))

        # get isleft or right corner
        isleft = corner_isleft[chosen_corner_index]

        # get offset from corner side (((CHECK THE SIGNS))) (((offset amount to be decided)))
        corner_x_offset = 0
        corner_y_offset = 0
        if (isleft):
            corner_x_offset = corner_offset
            corner_y_offset = -corner_offset
        else:
            corner_x_offset = -corner_offset
            corner_y_offset = corner_offset

        # calc next pos from corner + offset
        target_x = corner_x + corner_x_offset
        target_y = corner_y + corner_y_offset

        print("Target x = " + str(target_x))
        print("Target y = " + str(target_y))

        # THEN PUBLISH TARGET TO NEW MESSAGE TO BE READ BY OTHER

        wp = waypoint.Waypoint()
        wp.execute(target_y, target_x)

    # no corner found
    else:
        print("No corner found")


rospy.init_node('find_target_pos_edge')

odom_sub = message_filters.Subscriber('/odom', Odometry)
laser_sub = message_filters.Subscriber('/scan', LaserScan)

ts = message_filters.TimeSynchronizer([odom_sub, laser_sub], 10)
ts.registerCallback(getTargetPos)

#tf_sub = rospy.Subscriber('/tf', TFMessage, getTargetPos)
#laser_sub = rospy.Subscriber('/scan', LaserScan, getTargetPos)

# print(laser_sub)


rospy.spin()
