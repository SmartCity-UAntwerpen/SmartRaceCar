#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan


def location_2_pose(location):
    new_pose = Pose()
    new_pose.position.x = location.posx
    new_pose.position.y = location.posy
    new_pose.position.z = 0.0

    new_pose.orientation.x = 0.0
    new_pose.orientation.y = 0.0
    new_pose.orientation.z = location.orz
    new_pose.orientation.w = location.orw

    return new_pose


def pose_2_posestamped(pose):
    new_pose = PoseStamped()

    new_pose.header.stamp = rospy.Time.now()
    new_pose.header.frame_id = "map"

    new_pose.pose = pose

    return new_pose


def delegate_cost(startpose, goalpose, tolerance, speed):
    path = call_service_movebase(startpose, goalpose, tolerance)

    if path is not None:
        decimated_path = decimate_path(path.plan.poses)

        distance = get_distance(decimated_path)
        # logger.log_debug("[JAVALINKER][DELCOST] Distance: " + str(distance) + " meters")

        cost_time = get_time(distance, speed)
        # logger.log_debug("Estimated cost time: " + str(cost_time))

        return cost_time
    else:
        return None


def call_service_movebase(startpose, goalpose, tolerance):
    rospy.wait_for_service('move_base/make_plan')
    print "Waiting complete!"
    try:
        pathservice = rospy.ServiceProxy('move_base/make_plan', GetPlan)
        print "Serviceproxy made"

        calculated_path = pathservice(startpose, goalpose, tolerance)
        print "Path calculated"

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return None
    else:
        return calculated_path


def decimate_path(array_poses):
    decimated_array = array_poses[::5]
    return decimated_array


def get_distance(array_poses):
    distance = 0.0

    for x in range(1, len(array_poses)):
        pose_a = array_poses[x - 1]
        pose_b = array_poses[x]

        a_x = pose_a.pose.position.x
        a_y = pose_a.pose.position.y
        b_x = pose_b.pose.position.x
        b_y = pose_b.pose.position.y

        distance += math.sqrt(math.pow(b_x - a_x, 2) + math.pow(b_y - a_y, 2))

    print "Distance: %f" % distance
    return distance


def get_time(distance, speed):
    return distance / speed


# if __name__ == "__main__":
#     print "Starting"
#     rospy.init_node('Dummy_servicecaller', anonymous=True)
#
#     calculated_path = call_service_movebase()
#
#     if calculated_path is not None:
#         decimated_path = decimate_path(calculated_path.plan.poses)
#         distance = get_distance(decimated_path)
#         print "Estimated time: %f" % get_time(distance, 1.111111)
#
#     print "Done!"


def init_ros():
    # pub_drive_parameters = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    # pub_initial_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    # pub_movebase_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    rospy.init_node('rossimulator', anonymous=True)

    # rospy.Subscriber('move_base/status', GoalStatusArray, cb_movebase_status)
    # rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, cb_movebase_feedback)

    # logger.log_info("[ROSMODULE] ROS initialised")

'''
<x>0.5</x>
<y>0</y>
<z>-1</z>
<w>0.02</w>

<x>-13.4</x>
<y>-0.53</y>
<z>0.71</z>
<w>0.71</w>

<x>-27.14</x>
<y>-1.11</y>
<z>-0.3</z>
<w>0.95</w>

<x>-28.25</x>
<y>-9.19</y>
<z>-0.71</z>
<w>0.71</w>
'''