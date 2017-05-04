#!/usr/bin/env python

# Set this variable to False when using the Ros-system
# The code bypasses all Ros functions when set to True
DEBUG = True

import socket
import json
import logging
from threading import Thread
import time

if not DEBUG:
    import rospy
    from race.msg import drive_param
    from geometry_msgs.msg import PoseWithCovarianceStamped

    pub_drive_parameters = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    pub_initial_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('javalinker', anonymous=True)

TCP_IP = '127.0.0.1'
TCP_PORT_JAVA_PYTH = 5005
TCP_PORT_PYTH_JAVA = 5006
BUFFER_SIZE = 64

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

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
currentx = 0
currenty = 0
currentz = 0
currentw = 0


def publish_drive_param(json_string):
    if not DEBUG:
        msg = drive_param()
        msg.steer = json_string['drive']['steer']
        msg.throttle = json_string['drive']['throttle']
        rospy.loginfo(msg)
        pub_drive_parameters.publish(msg)
    return


def stop():
    if not DEBUG:
        msg = drive_param()
        msg.steer = 0
        msg.throttle = 0
        rospy.loginfo(msg)
        pub_drive_parameters.publish(msg)
    return
    return


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
    elif json_string.keys()[0] ==  'connect':
        connect()


def set_current_map(json_string):
    global startpointx,startpointy,startpointz,startpointw,meterperpixel
    startpointy = json_string['currentMap']['startPointY']
    startpointx =json_string['currentMap']['startPointX']
    startpointz = json_string['currentMap']['startPointZ']
    startpointw = json_string['currentMap']['startPointW']
    currentmap = json_string['currentMap']['name']
    meterperpixel = json_string['currentMap']['meterPerPixel']
    logging.info("Current map set: " + currentmap + " | startPoint: " + str(startpointx) + "," + str(startpointy) + "," + str(startpointz) + "," + str(startpointw) + " | MetersPerPixel: " + str(meterperpixel))


def set_next_waypoint(json_string):
    global nextwaypointx,nextwaypointy,nextwaypointz,nextwaypointw
    nextwaypointx = json_string['nextWayPoint']['x']
    nextwaypointy =json_string['nextWayPoint']['y']
    nextwaypointy = json_string['nextWayPoint']['z']
    nextwaypointy = json_string['nextWayPoint']['w']
    logging.info("Setting next waypoint: " + str(nextwaypointx) + "," + str(nextwaypointy) + "," + str(nextwaypointz) + "," + str(nextwaypointw))
    time.sleep(3)
    waypoint_reached()


def waypoint_reached():
    stop()
    global nextwaypointx,nextwaypointy,nextwaypointz,nextwaypointw
    logging.info("waypoint " + str(nextwaypointx) + "," + str(nextwaypointy) + "," + str(nextwaypointz) + "," + str(nextwaypointw) + " reached.")
    jsonmessage = {'arrivedWaypoint': {'x': nextwaypointx, 'y': nextwaypointy,'z': nextwaypointz, 'w': nextwaypointw}}
    json_string = json.dumps(jsonmessage)
    send_message(json_string)


def send_location():
    global currentx, currenty, currentz, currentw
    logging.info("Sending location: " + str(currentx) + "," + str(currenty) + "," + str(currentz) + "," + str(currentw))
    jsonmessage = {'location': {'x': currentx, 'y': currenty,'z':currentz,'w':currentw}}
    json_string = json.dumps(jsonmessage)
    send_message(json_string)


def send_message(json_string):
    global TCP_IP, TCP_PORT_PYTH_JAVA

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False;
    while not connected:
        try:
            client_socket.connect((TCP_IP, TCP_PORT_PYTH_JAVA))
            connected = True;
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


# rospy.Subscriber('drive_parameters', drive_param, callback)
newthread = ServerThread(TCP_IP, TCP_PORT_JAVA_PYTH, BUFFER_SIZE)
newthread.daemon = True
newthread.start()
if not DEBUG:
    rospy.spin()

while not connected:
    time.sleep(1)
while True:
    time.sleep(5)
    send_location()
