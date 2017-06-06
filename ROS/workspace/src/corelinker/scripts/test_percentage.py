#!/usr/bin/env python

import socket
import json
import rospy
import sys
import signal
import tf
from threading import Thread
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform
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


def cb_globalplan(data):
    global previous_secs, array_poses_pp_navfn, tf_mapbase
    if data.header.stamp.secs - previous_secs >= 1:
        previous_secs = data.header.stamp.secs
        #print "1 second past"
        array_poses_gp = data.poses
        array_poses_gp_new = []
        indexes = []
        # [(i, i + len(array_poses_gp)) for i in range(len(array_poses_pp_navfn)) if
        #    array_poses_pp_navfn[i:i + len(array_poses_gp)] == array_poses_gp]

        print "[TRANS] X: %.5f Y: %.5f Z: %.5f" % (tf_mapbase.posx, tf_mapbase.posy, tf_mapbase.posz)
        print "[ROTAT] X: %.5f Y: %.5f Z: %.5f W: %.5f" % (tf_mapbase.orx, tf_mapbase.ory, tf_mapbase.orz,
                                                           tf_mapbase.orw)

        for i in xrange(len(array_poses_gp)):
            loc = Location(array_poses_gp[i].pose.position.x + tf_mapbase.posx,
                           array_poses_gp[i].pose.position.y + tf_mapbase.posy,
                           array_poses_gp[i].pose.position.z + tf_mapbase.posz,
                           array_poses_gp[i].pose.orientation.x + tf_mapbase.orx,
                           array_poses_gp[i].pose.orientation.y + tf_mapbase.ory,
                           array_poses_gp[i].pose.orientation.z + tf_mapbase.orz,
                           array_poses_gp[i].pose.orientation.w + tf_mapbase.orw)

            array_poses_gp_new.append(loc)

        # for i in xrange(len(array_poses_gp)):
        #     print "---"
        #     print "X: %.5f Y: %.5f Z: %.5f" % (array_poses_gp[i].pose.position.x,
        #                                        array_poses_gp[i].pose.position.y,
        #                                        array_poses_gp[i].pose.position.z)
        #     print "NX: %.5f NY: %.5f NZ: %.5f" % (array_poses_gp_new[i].posx,
        #                                           array_poses_gp_new[i].posy,
        #                                           array_poses_gp_new[i].posz)
        #     print "TX: %.5f TY: %.5f TZ: %.5f" % (tf_mapodom.translation.x,
        #                                           tf_mapodom.translation.y,
        #                                           tf_mapodom.translation.z)
        #     print "-"
        #     print "X: %.5f Y: %.5f Z: %.5f W: %.5f" % (array_poses_gp[i].pose.orientation.x,
        #                                                array_poses_gp[i].pose.orientation.y,
        #                                                array_poses_gp[i].pose.orientation.z,
        #                                                array_poses_gp[i].pose.orientation.w)
        #     print "NX: %.5f NY: %.5f NZ: %.5f NW: %.5f" % (array_poses_gp_new[i].orx,
        #                                                    array_poses_gp_new[i].ory,
        #                                                    array_poses_gp_new[i].orz,
        #                                                    array_poses_gp_new[i].orw)
        #     print "-"
        #     print "%d / %d" % (i, len(array_poses_gp))

        # print "%d" % len(array_poses_gp)
        for i in range(len(array_poses_pp_navfn)):
            # if array_poses_pp_navfn[i:i + len(array_poses_gp_new)] == array_poses_gp:
            temp_array = array_poses_pp_navfn[i:i + len(array_poses_gp_new)]
            if check_equality_plan_global(temp_array[0].pose, array_poses_gp_new[0]):
                print "Heuj!"


def check_equality_plan_global(plannerplan_coord, globalplan_coord):
    if globalplan_coord.posx == plannerplan_coord.position.x:
        print "X correct"
        if globalplan_coord.posy == plannerplan_coord.position.y:
            print "Y correct"
            if globalplan_coord.posz == plannerplan_coord.position.z:
                print "Complete correct!"
                return True

    return False


def loc_apply_transform(pose, transform):
    loc = Location(pose.position.x + transform.translation.x,
                   pose.position.y + transform.translation.y,
                   pose.position.z + transform.translation.z,
                   pose.orientation.x + transform.rotation.x,
                   pose.orientation.y + transform.rotation.y,
                   pose.orientation.z + transform.rotation.z,
                   pose.orientation.w + transform.rotation.w)


def cb_transform(data):
    global tf_mapodom, tf_odombase

    tf = data.transforms[0]
    if tf.header.frame_id == "map":
        tf_mapodom.translation.x = tf.transform.translation.x
        tf_mapodom.translation.y = tf.transform.translation.y
        tf_mapodom.translation.z = tf.transform.translation.z

        tf_mapodom.rotation.x = tf.transform.rotation.x
        tf_mapodom.rotation.y = tf.transform.rotation.y
        tf_mapodom.rotation.z = tf.transform.rotation.z
        tf_mapodom.rotation.w = tf.transform.rotation.w

        # print "---"
        # print "X: %.2f, Y: %.2f, Z: %.2f" % (tf_mapodom.translation.x, tf_mapodom.translation.y,
        #                                      tf_mapodom.translation.z)
        # print "X: %.2f, Y: %.2f, Z: %.2f, W: %.2f" % (tf_mapodom.rotation.x, tf_mapodom.rotation.y,
        #                                               tf_mapodom.rotation.z, tf_mapodom.rotation.w)

    if tf.header.frame_id == "odom":
        tf_odombase.translation.x = tf.transform.translation.x
        tf_odombase.translation.y = tf.transform.translation.y
        tf_odombase.translation.z = tf.transform.translation.z

        tf_odombase.rotation.x = tf.transform.rotation.x
        tf_odombase.rotation.y = tf.transform.rotation.y
        tf_odombase.rotation.z = tf.transform.rotation.z
        tf_odombase.rotation.w = tf.transform.rotation.w


def cb_localplan(data):
    # print "local plan received!"
    pass

rospy.Subscriber('/move_base/NavfnROS/plan', Path, cb_plannerplan_navfn)
rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, cb_plannerplan_gp)
rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, cb_globalplan)
rospy.Subscriber('/move_base/TrajectoryPlannerROS/local_plan', Path, cb_localplan)
# rospy.Subscriber('/tf', TFMessage, cb_transform)


class TFThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.stop_thread = False

    def run(self):
        global tf_mapbase

        print "Starting Thread"
        listener = tf.TransformListener()
        rate = rospy.Rate(2.0)
        while not self.stop_thread:
            try:
                (trans, rot) = listener.lookupTransform('/map', '/base_frame', rospy.Time(0))
                # print "[TRANS] X: %.5f Y: %.5f Z: %.5f" % (trans[0], trans[1], trans[2])
                # print "[ROTAT] X: %.5f Y: %.5f Z: %.5f W: %.5f" % (rot[0], rot[1], rot[2], rot[3])
                tf_mapbase = Location(trans[0], trans[1], trans[2],
                                      rot[0], rot[1], rot[2], rot[3])
            except:
                print "Error with lookuptransform"
                continue

    def stop_thread(self):
        self.stop_thread = True


def signal_handler(signal, frame):
    global thread_tf
    thread_tf.stop_thread()
    sys.exit(0)


thread_tf = TFThread()
thread_tf.daemon = True
thread_tf.start()

signal.signal(signal.SIGINT, signal_handler)

rospy.spin()

