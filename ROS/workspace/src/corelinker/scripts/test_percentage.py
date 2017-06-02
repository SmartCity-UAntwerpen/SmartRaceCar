#!/usr/bin/env python

import socket
import json
import rospy
from threading import Thread
from nav_msgs.msg import Path
import handlers.calc_cost as calccost

rospy.init_node('javalinker', anonymous=True)

array_poses_pp_navfn = None
planner_distance = None

previous_secs = 0


def cb_plannerplan_navfn(data):
    global array_poses_pp_navfn, planner_distance

    array_poses_pp_navfn = data.poses
    print "[NAVFN] Derived poses from data"

    decimated_poses = calccost.decimate_path(array_poses_pp_navfn)
    print "[NAVFN] Decimated path"

    planner_distance = calccost.get_distance(decimated_poses)
    print "[NAVFN] Distance: " + str(planner_distance)


def cb_plannerplan_gp(data):
    print "GlobalPlanner received!"


def cb_globalplan(data):
    global previous_secs, array_poses_pp_navfn
    # if data.header.stamp.secs - previous_secs >= 1:
    if 1:
        # previous_secs = data.header.stamp.secs
        #print "1 second past"
        array_poses_gp = data.poses
        indexes = []
        # [(i, i + len(array_poses_gp)) for i in range(len(array_poses_pp_navfn)) if
        #    array_poses_pp_navfn[i:i + len(array_poses_gp)] == array_poses_gp]

        print "%d" % len(array_poses_gp)
        for i in range(len(array_poses_pp_navfn)):
            if array_poses_pp_navfn[i:i + len(array_poses_gp)] == array_poses_gp:
                print "kak"
                indexes.append((i, i + len(array_poses_gp)))
        print indexes

    pass


def cb_localplan(data):
    # print "local plan received!"
    pass

rospy.Subscriber('/move_base/NavfnROS/plan', Path, cb_plannerplan_navfn)
rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, cb_plannerplan_gp)
rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, cb_globalplan)
rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, cb_localplan)

rospy.spin()
