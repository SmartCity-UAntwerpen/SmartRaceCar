#!/usr/bin/env python

import socket
import time
import json
import rospy
from threading import Thread
from race.msg import drive_param


TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024


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
        (conn, (ip, port)) = self.server_socket.accept()
        while True:
            data = conn.recv(self._BUFFER_)
            j = json.loads(data)
            print "Server received data: " + str(j['drive']['throttle'])


class ClientThread(Thread):
    def __init__(self, _IP_, _PORT_, _BUFFER_):
        print "[ ] Initializing clientthread"
        Thread.__init__(self)
        self._IP_ = _IP_
        self._PORT_ = _PORT_
        self._BUFFER_ = _BUFFER_

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print "[+] Initializing clientthread finished"

    def run(self):
        print "Starting clientthread"
        self.client_socket.connect((self._IP_, self._PORT_))

        i = 0
        while i < 10:
            jsonmessage = {'drive': {'steer': i, 'throttle': (i * 2)}}
            string = json.dumps(jsonmessage)
            self.client_socket.send(string)
            i += 1
            time.sleep(0.001)


def callback(data):
    rospy.loginfo(data)

rospy.init_node('javalinker', anonymous=True)
rospy.Subscriber('drive_parameters', drive_param, callback)

newthread = ServerThread(TCP_IP, TCP_PORT, BUFFER_SIZE)
newthread.daemon = True
newthread.start()

newthread2 = ClientThread(TCP_IP, TCP_PORT, BUFFER_SIZE)
newthread2.daemon = True
newthread2.start()

rospy.spin()

print "newthread2 finished"