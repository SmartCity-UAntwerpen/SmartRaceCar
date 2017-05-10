#!/usr/bin/env python

# Set this variable to False when using the Ros-system
# The code bypasses all Ros functions when set to True
DEBUG_WITHOUT_ROS = False
DEBUG_WITHOUT_JAVA = True

import socket
import json
import handlers.logger as logger
import logging
from threading import Thread
import time
import handlers.ros_class as rosmodule
from handlers.location import Location

logger = logger.Logger()

if not DEBUG_WITHOUT_ROS:
    rosmodule.init_ros(logger)
    logger.log_debug("[JAVALINKER] Debug with ros!")

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



def read_line(sock, recv_buffer=4096, delim='\n'):
    line_buffer = ''
    data = True
    while data:
        data = sock.recv(recv_buffer)
        line_buffer += data

    while line_buffer.find(delim) != -1:
        line, line_buffer = line_buffer.split('\n', 1)
        yield line
    return


def get_type(json_string):
    if json_string.keys()[0] == 'drive':
        publish_drive_param(json_string)
    elif json_string.keys()[0] == 'currentMap':
        set_current_map(json_string)
    elif json_string.keys()[0] == 'nextWayPoint':
        set_next_waypoint(json_string)
    elif json_string.keys()[0] == 'connect':
        connect()
    elif json_string.keys()[0] == 'startPoint':
        set_startpoint(json_string)


def set_current_map(json_string):
    global currentmap, meterperpixel
    currentmap = json_string['currentMap']['name']
    meterperpixel = json_string['currentMap']['meterPerPixel']
    logging.info("Current map set: " + currentmap + " | MetersPerPixel: " + str(meterperpixel))


def set_startpoint(json_string):
    global startpointx, startpointy, startpointz, startpointw
    startpointx = json_string['startPoint']['x']
    startpointy = json_string['startPoint']['y']
    startpointz = json_string['startPoint']['z']
    startpointw = json_string['startPoint']['w']
    logging.info("Current startPoint set: " + str(startpointx) + "," + str(startpointy) + "," + str(startpointz) + ","
                 + str(startpointw))
    rosmodule.publish_initialpose(startpointx, startpointy, 0.0, 0.0, 0.0, startpointz, startpointw)


def set_next_waypoint(json_string):
    global nextwaypointx, nextwaypointy, nextwaypointz, nextwaypointw
    nextwaypointx = json_string['nextWayPoint']['x']
    nextwaypointy = json_string['nextWayPoint']['y']
    nextwaypointz = json_string['nextWayPoint']['z']
    nextwaypointw = json_string['nextWayPoint']['w']
    logging.info("Setting next waypoint: " + str(nextwaypointx) + "," + str(nextwaypointy) + "," + str(nextwaypointz) +
                 "," + str(nextwaypointw))
    rosmodule.publish_movebase_goal(nextwaypointx, nextwaypointy, 0.0, 0.0, 0.0, nextwaypointz, nextwaypointw)
    time.sleep(3)
    waypoint_reached()


def waypoint_reached():
    rosmodule.stop()
    global nextwaypointx, nextwaypointy, nextwaypointz, nextwaypointw
    logging.info("waypoint " + str(nextwaypointx) + "," + str(nextwaypointy) + "," + str(nextwaypointz) + "," +
                 str(nextwaypointw) + " reached.")
    jsonmessage = {'arrivedWaypoint': {'x': nextwaypointx, 'y': nextwaypointy, 'z': nextwaypointz, 'w': nextwaypointw}}
    json_string = json.dumps(jsonmessage)
    send_message(json_string)


def send_location(location):
    logging.info("Sending location: posx: " + str(location.posx) + ", posy: " + str(location.posy) + ", orz: " +
                 str(location.orz) + ", orw: " + str(location.orw))
    jsonmessage = {'location': {'x': location.posx, 'y': location.posy, 'z': location.orz, 'w': location.orw}}
    json_string = json.dumps(jsonmessage)
    send_message(json_string)


def send_message(json_string):
    global TCP_IP, TCP_PORT_PYTH_JAVA, connected

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False
    while not connected:
        try:
            client_socket.connect((TCP_IP, TCP_PORT_PYTH_JAVA))
            connected = True
        except socket.error, exc:
            logging.warning("[CLIENTSOCKET] Cannot send data:  " + json_string + "   Trying again." + str(exc))
            connected = False
            time.sleep(1)
    logging.debug("[CLIENTSOCKET] Socket created & connected")

    client_socket.sendall(json_string)
    logging.debug("[CLIENTSOCKET] data sent: " + json_string)

    client_socket.close()
    logging.debug("[CLIENTSOCKET] Socket closed")


def connect():
    jsonmessage = {'connect': {'x': 0, 'y': 0}}
    json_string = json.dumps(jsonmessage)
    send_message(json_string)
    logging.info("Connected to Core.")
    global connected
    connected = True


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
            data = read_line(conn)
            data_string = str("".join(data))
            logging.debug("[SERVERSOCKET] Server received data: " + data_string)
            json_string = json.loads(data_string)
            get_type(json_string)
            # print json_string.keys()[0]


def callback(data):
    global TCP_IP, TCP_PORT_PYTH_JAVA

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((TCP_IP, TCP_PORT_PYTH_JAVA))
    logging.debug("[CLIENTSOCKET] Socket created & connected")

    jsonmessage = {'drive': {'steer': data.angle, 'throttle': data.velocity}}
    string = json.dumps(jsonmessage)

    client_socket.sendall(string)
    logging.debug("[CLIENTSOCKET] data sent: " + string)

    client_socket.close()
    logging.debug("[CLIENTSOCKET] Socket closed")


if not DEBUG_WITHOUT_JAVA:
    newthread = ServerThread(TCP_IP, TCP_PORT_JAVA_PYTH, BUFFER_SIZE)
    newthread.daemon = True
    newthread.start()

if not DEBUG_WITHOUT_JAVA:
    while not connected:
        time.sleep(1)

if not DEBUG_WITHOUT_ROS:
    rosmodule.rospy_spin()
else:
    while True:
        time.sleep(5)
        send_location()
