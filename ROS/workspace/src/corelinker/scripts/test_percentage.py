#!/usr/bin/env python

import socket
import json
import rospy
import sys
import signal
from threading import Thread

import tf
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped

import handlers.calc_cost as calccost
from handlers.location import Location

rospy.init_node('javalinker', anonymous=True)

array_poses_pp_navfn = None
planner_distance = None

previous_secs = 0

tf_mapbase = None


def cb_plannerplan_navfn(data):
    print "NavfnROS received!"
    handler_plannerplan(data)


def cb_plannerplan_gp(data):
    print "GlobalPlanner received!"
    handler_plannerplan(data)


def handler_plannerplan(data):
    global array_poses_pp_navfn, planner_distance

    array_poses_pp_navfn = data.poses
    print "[NAVFN] Derived poses from data"

    decimated_poses = calccost.decimate_path(array_poses_pp_navfn)
    print "[NAVFN] Decimated path"

    planner_distance = calccost.get_distance(decimated_poses)
    print "[NAVFN] Distance: " + str(planner_distance)


def cb_feedback(data):
    point = data.pose.pose.position

    dec_orig_array = calccost.decimate_path(array_poses_pp_navfn)
    dist_orig_array = calccost.get_distance(dec_orig_array)

    index_array = calc_closest_point(array_poses_pp_navfn, point)

    interm_array = array_poses_pp_navfn[index_array:]
    dec_interm_array= calccost.decimate_path(interm_array)
    dist_interm_array = calccost.get_distance(dec_interm_array)
    print "[FEEDBACK] Distance: %.5f" % dist_interm_array

    percentage = (dist_orig_array - dist_interm_array) / dist_orig_array * 100
    print "[FEEDBACK] Percentage: %d%%" % percentage

def calc_closest_point(path, point):
    precision = find_precision(len(path))
    print "[PRECISION] %d" % precision

    (best, best_length, best_distance) = coarse_approximation(precision, path, point)
    print "[COARSE][BEST] X: %.5f Y: %.5f - Length: %d - Dist: %.5f" % (best.x, best.y, best_length, best_distance)

    (best, best_length, best_distance) = accurate_approximation(precision, path, point, best, best_length,
                                                                best_distance)
    print "[ACCURATE][BEST] X: %.5f Y: %.5f - Length: %d - Dist: %.5f" % (best.x, best.y, best_length, best_distance)

    return best_length


def find_precision(total_length):
    stop = False
    i = 1
    while not stop:
        if i * 2 * 100 > total_length:
            stop = True
        else:
            i *= 2

    return i


def coarse_approximation(precision, path, point):
    path_length = len(path)
    scan_length = 0

    best = -1
    best_length = -1
    best_distance = float("inf")

    while scan_length <= path_length:
        scan = path[scan_length].pose.position
        scan_distance = calc_distance_2_point(scan.x, scan.y, point.x, point.y)

        if scan_distance < best_distance:
            best = scan
            best_length = scan_length
            best_distance = scan_distance
        scan_length += precision
    return best, best_length, best_distance


def accurate_approximation(precision, path, point, best, best_length, best_distance):
    precision /= 2

    while precision > 0.5:

        # Assignment of the variables used in the comparisons
        before_length = best_length - precision
        if before_length >= 0:
            before = path[before_length].pose.position
            before_distance = calc_distance_2_point(before.x, before.y, point.x, point.y)

        after_length = best_length + precision
        if after_length < len(path):
            after = path[after_length].pose.position
            after_distance = calc_distance_2_point(after.x, after.y, point.x, point.y)

        # The actual comparisons
        if before_length >= 0 and before_distance < best_distance:
            best = before
            best_length = before_length
            best_distance = before_distance
        elif after_length < len(path) and after_distance < best_distance:
            best = after
            best_length = after_length
            best_distance = after_distance
        else:
            precision /= 2

    return best, best_length, best_distance


def calc_distance_2_point(x_path, y_path, x_point, y_point):
    dx = x_path - x_point
    dy = y_path - y_point
    return dx * dx + dy * dy


rospy.Subscriber('/move_base/NavfnROS/plan', Path, cb_plannerplan_navfn)
rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, cb_plannerplan_gp)
# rospy.Subscriber('/move_base/feedback', Path, cb_feedback)
rospy.Subscriber('initialpose', PoseWithCovarianceStamped, cb_feedback)
# rospy.Subscriber('/tf', TFMessage, cb_transform)

rospy.spin()

