#!/usr/bin/env python

import socket
import json
import rospy
from threading import Thread
from race.msg import drive_param

TCP_IP = '127.0.0.1'
TCP_PORT_JAVA_PYTH = 5005
TCP_PORT_PYTH_JAVA = 5006
BUFFER_SIZE = 64

# pub_drive_parameters = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
rospy.init_node('javalinker', anonymous=True)


def publish_drive_param(json_string):
    if json_string.keys()[0] == 'drive':
        msg = drive_param()
        msg.angle = json_string['drive']['steer']
        msg.velocity = json_string['drive']['throttle']
        rospy.loginfo(msg)
        # pub_drive_parameters.publish(msg)


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


class ServerThread(Thread):
    def __init__(self, _IP_, _PORT_, _BUFFER_):
        print "[ ] Initializing serverthread"
        Thread.__init__(self)
        self._IP_ = _IP_
        self._PORT_ = _PORT_
        self._BUFFER_ = _BUFFER_

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self._IP_, self._PORT_))
        print "[+] Initializing serverthread finished"

    def run(self):
        print "Starting serverthread"
        self.server_socket.listen(4)

        while True:
            (conn, (ip, port)) = self.server_socket.accept()
            data = read_line(conn)
            data_string = str("".join(data))
            print "Server received data: " + data_string
            json_string = json.loads(data_string)
            publish_drive_param(json_string)
            # print json_string.keys()[0]


def callback(data):
    print "kaka"
    global TCP_IP, TCP_PORT_PYTH_JAVA

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((TCP_IP, TCP_PORT_PYTH_JAVA))
    print "[CLIENTSOCKET] Socket created & connected"

    jsonmessage = {'drive': {'steer': data.angle, 'throttle': data.velocity}}
    string = json.dumps(jsonmessage)
    print "[CLIENTSOCKET] Message composed"

    client_socket.sendall(string)
    print "[CLIENTSOCKET] String sent"

    client_socket.close()
    print "[CLIENTSOCKET] Socket closed"


rospy.Subscriber('drive_parameters', drive_param, callback)

newthread = ServerThread(TCP_IP, TCP_PORT_JAVA_PYTH, BUFFER_SIZE)
newthread.daemon = True
newthread.start()

rospy.spin()

print "newthread2 finished"
