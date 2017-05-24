#!/usr/bin/env python

import handlers.logger as logmodule
import json
import os
import time
import handlers.java_module as javamodule
import handlers.threads as threadsmodule
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
    if json_string.keys()[0] == 'currentMap':
        logger.log_debug("[JAVALINKER] json_string key = currentMap")
        set_current_map(json_string)
    elif json_string.keys()[0] == 'connect':
        logger.log_debug("[JAVALINKER] json_string key = connect")
        javamodule.connect()
    elif json_string.keys()[0] == 'cost':
        logger.log_debug("[JAVALINKER] json_string key = cost")
        calculate_cost(json_string)


def set_current_map(json_string):
    global currentmap, logger
    currentmap = json_string['currentMap']['name']
    logger.log_info("Current map set: " + currentmap)

    launch_navstack(currentmap)


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


"""
######################
##  Main functions  ##
######################
"""


def launch_navstack(curmap):
    threadsmodule.start_navstack_thread(curmap, navstack_speed)
    logger.log_debug("[JAVALINKER] navstack launched")

if __name__ == "__main__":
    if not DEBUG_WITHOUT_JAVA:
        javamodule.set_logger(logger)
        start_thread()
        logger.log_info("Debug without java: False")

    if not DEBUG_WITHOUT_ROS:
        rosmodule.init_ros(logger)
        logger.log_debug("[JAVALINKER] Debug with ros!")

        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from move_base_msgs.msg import MoveBaseActionFeedback

        rosmodule.rospy_spin()

    else:
        while True:
            time.sleep(5)
