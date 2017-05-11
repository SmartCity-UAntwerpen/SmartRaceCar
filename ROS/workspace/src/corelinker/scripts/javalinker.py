#!/usr/bin/env python

import handlers.logger as logmodule
import json
import time
import handlers.java_module as javamodule
import handlers.ros_module as rosmodule
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


def set_current_map(json_string):
    global currentmap, logger
    currentmap = json_string['currentMap']['name']
    logger.log_info("Current map set: " + currentmap)


def set_startpoint(json_string):
    start_location = Location(json_string['startPoint']['x'], json_string['startPoint']['y'], 0.0, 0.0, 0.0,
                              json_string['startPoint']['z'], json_string['startPoint']['w'])

    logger.log_info("Current startPoint set: " + str(start_location.posx) + "," + str(start_location.posy) + "," +
                    str(start_location.orz) + "," + str(start_location.orw))
    rosmodule.publish_initialpose(start_location)


def set_next_waypoint(json_string):
    print json_string
    next_waypoint = Location(json_string['nextWayPoint']['x'], json_string['nextWayPoint']['y'], 0.0, 0.0, 0.0,
                             json_string['nextWayPoint']['z'], json_string['nextWayPoint']['w'])
    logger.log_info("Setting next waypoint: " + str(next_waypoint.posx) + "," + str(next_waypoint.posy) + "," +
                    str(next_waypoint.orz) + "," + str(next_waypoint.orw))
    rosmodule.publish_movebase_goal(next_waypoint.posx, next_waypoint.posy, 0.0, 0.0, 0.0, next_waypoint.orz,
                                    next_waypoint.orw)
    # time.sleep(3)
    # waypoint_reached()


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
##############################
##  ROS Callback functions  ##
##############################
"""


def cb_movebase_status(data):
    status_list = data.status_list
    if len(status_list) != 0:
        status = status_list[len(status_list) - 1].status
        logger.log_debug("[STATUS] Status: " + status_list[len(status_list) - 1].status)

        if status != cb_movebase_status_previous:
            if status == 3:
                logger.log_debug("[JAVALINKER] Waypoint reached")
                waypoint_reached()


def cb_movebase_feedback(data):
    global cb_movebase_feedback_secs
    header = data.header
    if header.stamp.secs - cb_movebase_feedback_secs >= 1:
        cb_movebase_feedback_secs = header.stamp.secs
        pose = data.feedback.base_position.pose
        logger.log_debug("[FEEDBACK] Secs: " + header.stamp.secs)
        logger.log_debug("[FEEDBACK] Position: X: " + pose.position.x +
                         ", Y: " + pose.position.y +
                         ", Z: " + pose.position.z)
        logger.log_debug("[FEEDBACK] Orientation: X: " + pose.orientation.x +
                         ", Y: " + pose.orientation.y +
                         ", Z: " + pose.orientation.z +
                         ", W: " + pose.orientation.w)
        current_location = Location(pose.position.x, pose.position.y, 0, 0, 0, pose.orientation.z, pose.orientation.w)
        send_location(current_location)

"""
######################
##  Main functions  ##
######################
"""

if __name__ == "__main__":
    if not DEBUG_WITHOUT_JAVA:
        javamodule.set_logger(logger)
        start_thread()
        logger.log_info("Debug without java: False")

    if not DEBUG_WITHOUT_ROS:
        rosmodule.init_ros(logger)
        logger.log_debug("[JAVALINKER] Debug with ros!")
        rosmodule.rospy_spin()

        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from move_base_msgs.msg import MoveBaseActionFeedback

        rospy.Subscriber('move_base/status', GoalStatusArray, cb_movebase_status)
        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, cb_movebase_feedback)

    else:
        while True:
            time.sleep(5)
