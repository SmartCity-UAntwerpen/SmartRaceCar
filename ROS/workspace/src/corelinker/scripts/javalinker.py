#!/usr/bin/env python

import handlers.logger as logmodule
import json
import os
import time
import handlers.java_module as javamodule
import handlers.ros_module as rosmodule
import handlers.calc_cost as calccost
from handlers.location import Location
import socket
from threading import Thread

logger = logmodule.Logger()

# Set this variable to False when using the Ros-system
# The code bypasses all Ros functions when set to True
DEBUG_WITHOUT_ROS = False
DEBUG_WITHOUT_JAVA = False

TCP_IP = '127.0.0.1'
TCP_PORT_JAVA_PYTH = 5005
TCP_PORT_PYTH_JAVA = 5006
BUFFER_SIZE = 64

connected = False
currentmap = 'default'
startpointx = 0
startpointy = 0
startpointz = 0
startpointw = 0
meterperpixel = 0
nextwaypointx = 0
nextwaypointy = 0
nextwaypointz = 0
nextwaypointw = 0
currentx = 1
currenty = 2
currentz = 3
currentw = 4

navplan_tolerance = 0.5
navplan_speed = 4.0

navstack_speed = 4.0

current_location = 0

cb_movebase_feedback_secs = 0
cb_movebase_status_previous = 0

time.sleep(1)
location = Location(2, 3, 0, 0, 0, 4, 4)


"""
############################
##  Delegation functions  ##
############################
"""


def get_type(json_string):
    if json_string.keys()[0] == 'drive':
        logger.log_debug("[JAVALINKER] json_string key = drive")
        rosmodule.publish_drive_param(json_string)
    elif json_string.keys()[0] == 'currentMap':
        logger.log_debug("[JAVALINKER] json_string key = currentMap")
        set_current_map(json_string)
    elif json_string.keys()[0] == 'nextWayPoint':
        logger.log_debug("[JAVALINKER] json_string key = nextWayPoint")
        set_next_waypoint(json_string)
    elif json_string.keys()[0] == 'connect':
        logger.log_debug("[JAVALINKER] json_string key = connect")
        javamodule.connect()
    elif json_string.keys()[0] == 'startPoint':
        logger.log_debug("[JAVALINKER] json_string key = startPoint")
        set_startpoint(json_string)
    elif json_string.keys()[0] == 'cost':
        logger.log_debug("[JAVALINKER] json_string key = cost")
        calculate_cost(json_string)


def set_current_map(json_string):
    global currentmap, logger
    currentmap = json_string['currentMap']['name']
    logger.log_info("Current map set: " + currentmap)

    launch_navstack(currentmap)


def set_startpoint(json_string):
    global current_location
    start_location = Location(json_string['startPoint']['x'], json_string['startPoint']['y'], 0.0, 0.0, 0.0,
                              json_string['startPoint']['z'], json_string['startPoint']['w'])

    logger.log_info("Current startPoint set: " + str(start_location.posx) + "," + str(start_location.posy) + "," +
                    str(start_location.orz) + "," + str(start_location.orw))
    current_location = start_location

    rosmodule.publish_initialpose(start_location)


def set_next_waypoint(json_string):
    next_waypoint = Location(json_string['nextWayPoint']['x'], json_string['nextWayPoint']['y'], 0.0, 0.0, 0.0,
                             json_string['nextWayPoint']['z'], json_string['nextWayPoint']['w'])
    logger.log_info("Setting next waypoint: " + str(next_waypoint.posx) + "," + str(next_waypoint.posy) + "," +
                    str(next_waypoint.orz) + "," + str(next_waypoint.orw))
    rosmodule.publish_movebase_goal(next_waypoint)


def waypoint_reached():
    rosmodule.stop()
    # logger.log_info("waypoint " + str(nextwaypointx) + "," + str(nextwaypointy) + "," + str(nextwaypointz) + "," +
    #                 str(nextwaypointw) + " reached.")

    # Coordinates from arrived waypoint pure for stuffing
    jsonmessage = {'arrivedWaypoint': {'x': 0, 'y': 0, 'z': 0, 'w': 0}}
    json_string = json.dumps(jsonmessage)
    javamodule.send_message(json_string)
    logger.log_debug("[JAVALINKER] Waypoint_reached sent")


def send_location(location):
    logger.log_info("Sending location: posx: " + str(location.posx) + ", posy: " + str(location.posy) + ", orz: " +
                    str(location.orz) + ", orw: " + str(location.orw))
    jsonmessage = {'location': {'x': location.posx, 'y': location.posy, 'z': location.orz, 'w': location.orw}}
    json_string = json.dumps(jsonmessage)
    javamodule.send_message(json_string)


def calculate_cost(json_string):
    global current_location

    start_location = Location(json_string['cost'][0]['x'], json_string['cost'][0]['y'], 0.0, 0.0, 0.0,
                              json_string['cost'][0]['z'], json_string['cost'][0]['w'])
    goal_location = Location(json_string['cost'][1]['x'], json_string['cost'][1]['y'], 0.0, 0.0, 0.0,
                             json_string['cost'][1]['z'], json_string['cost'][1]['w'])

    current_posestamped = rosmodule.pose_2_posestamped(rosmodule.location_2_pose(current_location))
    start_posestamped = rosmodule.pose_2_posestamped(rosmodule.location_2_pose(start_location))
    goal_posestamped = rosmodule.pose_2_posestamped(rosmodule.location_2_pose(goal_location))

    logger.log_debug(current_posestamped)
    logger.log_debug(start_posestamped)
    logger.log_debug(goal_posestamped)

    costtime_current_start = delegate_cost(current_posestamped, start_posestamped, navplan_tolerance, navplan_speed)
    logger.log_debug("[JAVALINKER][CALCCOST] Cost current-start: " + str(costtime_current_start) + " seconds")

    costtime_start_goal = delegate_cost(start_posestamped, goal_posestamped, navplan_tolerance, navplan_speed)
    logger.log_debug("[JAVALINKER][CALCCOST] Cost start-goal: " + str(costtime_start_goal) + " seconds")

    jsonmessage = {'cost': {'status': False, 'weightToStart': costtime_current_start,
                            'weight': costtime_start_goal, 'idVehicle': 12321}}
    logger.log_debug(json_string)
    json_string = json.dumps(jsonmessage)
    javamodule.send_message(json_string)


def delegate_cost(startpose, goalpose, tolerance, speed):
    path = calccost.call_service_movebase(startpose, goalpose, tolerance)

    if path is not None:
        decimated_path = calccost.decimate_path(path.plan.poses)

        distance = calccost.get_distance(decimated_path)
        logger.log_debug("[JAVALINKER][DELCOST] Distance: " + str(distance) + " meters")

        cost_time = calccost.get_time(distance, speed)
        logger.log_debug("Estimated cost time: " + str(cost_time))

        return cost_time
    else:
        return None


"""
#############################
##  Incoming TCP-requests  ##
#############################
"""


def start_thread():
    newthread = ServerThread(TCP_IP, TCP_PORT_JAVA_PYTH, BUFFER_SIZE)
    newthread.daemon = True
    newthread.start()


class ServerThread(Thread):
    def __init__(self, _IP_, _PORT_, _BUFFER_):
        Thread.__init__(self)
        self._IP_ = _IP_
        self._PORT_ = _PORT_
        self._BUFFER_ = _BUFFER_

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self._IP_, self._PORT_))

    def run(self):
        self.server_socket.listen(4)
        while True:
            (conn, (ip, port)) = self.server_socket.accept()
            data = javamodule.read_line(conn)
            data_string = str("".join(data))
            logger.log_debug("[SERVERSOCKET] Server received data: " + data_string)
            json_string = json.loads(data_string)
            get_type(json_string)


def start_navstack_thread(currentmap, speed):
    newthread = NavStack_Thread(currentmap, speed)
    newthread.daemon = True
    newthread.start()


class NavStack_Thread(Thread):
    def __init__(self, _CURRENTMAP_, _SPEED_):
        Thread.__init__(self)
        self._CURRENTMAP_ = _CURRENTMAP_
        self._SPEED_ = _SPEED_

    def run(self):
        command = "roslaunch f1tenth_2dnav move_base.launch map_name:=" + self._CURRENTMAP_ + \
                  ".yaml speed:=" + self._SPEED_ + ""
        os.system(command)

"""
##############################
##  ROS Callback functions  ##
##############################
"""


def cb_movebase_status(data):
    global cb_movebase_status_previous

    status_list = data.status_list
    if len(status_list) != 0:
        status = status_list[len(status_list) - 1].status
        logger.log_debug("[STATUS] Status: " + status_list[len(status_list) - 1].status)

        if status != cb_movebase_status_previous:
            cb_movebase_status_previous = status
            if status == 3:
                logger.log_debug("[JAVALINKER] Waypoint reached")
                waypoint_reached()


def cb_movebase_feedback(data):
    global cb_movebase_feedback_secs, current_location
    header = data.header
    if header.stamp.secs - cb_movebase_feedback_secs >= 1:
        cb_movebase_feedback_secs = header.stamp.secs
        pose = data.feedback.base_position.pose
        logger.log_debug("[FEEDBACK] Secs: " + str(header.stamp.secs))
        logger.log_debug("[FEEDBACK] Position: X: " + str(pose.position.x) +
                         ", Y: " + str(pose.position.y) +
                         ", Z: " + str(pose.position.z))
        logger.log_debug("[FEEDBACK] Orientation: X: " + str(pose.orientation.x) +
                         ", Y: " + str(pose.orientation.y) +
                         ", Z: " + str(pose.orientation.z) +
                         ", W: " + str(pose.orientation.w))
        current_location = Location(pose.position.x, pose.position.y, 0, 0, 0, pose.orientation.z, pose.orientation.w)
        send_location(current_location)

"""
######################
##  Main functions  ##
######################
"""


def launch_navstack(curmap):
    start_navstack_thread(curmap, navstack_speed)
    logger.log_debug("[JAVALINKER] navstack launched")

if __name__ == "__main__":
    if not DEBUG_WITHOUT_JAVA:
        javamodule.set_logger(logger)
        start_thread()
        logger.log_info("Debug without java: False")

    # while currentmap is 'default':
    #     print "Waiting for map"
    #     time.sleep(0.1)
    # os.system("roslaunch f1tenth_2dnav move_base.launch map_name:=zbuilding.yaml speed:=1.4 ")

    if not DEBUG_WITHOUT_ROS:
        rosmodule.init_ros(logger)
        logger.log_debug("[JAVALINKER] Debug with ros!")

        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from move_base_msgs.msg import MoveBaseActionFeedback

        rospy.Subscriber('move_base/status', GoalStatusArray, cb_movebase_status)
        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, cb_movebase_feedback)
        rosmodule.rospy_spin()

    else:
        while True:
            time.sleep(5)
