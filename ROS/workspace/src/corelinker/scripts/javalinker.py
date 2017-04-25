#!/usr/bin/env python

import socket
import time
import json
import rospy
from threading import Thread
from race.msg import drive_param


TCP_IP = '127.0.0.1'
TCP_PORT_JAVA_PYTH = 5005
TCP_PORT_PYTH_JAVA = 5006
BUFFER_SIZE = 64

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# pub_drive_parameters = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
rospy.init_node('javalinker', anonymous=True)

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
            data = self.readline(conn)
            data_string = str("".join(data))
            print "Server received data: " + data_string
            #pis = ":".join("{:02x}".format(ord(c)) for c in str(data))
            #print pis
            json_string = json.loads(data_string)
            self.publish(json_string)
            # print json_string.keys()[0]

    def publish(self, json_string):
        if json_string.keys()[0] == 'drive':
            msg = drive_param()
            # Set message parameters
            msg.angle = json_string['drive']['steer']
            msg.velocity = json_string['drive']['throttle']
            rospy.loginfo(msg)
            #pub_drive_parameters.publish(msg)

    def readline(self, sock, recv_buffer=4096, delim='\n'):
        buffer = ''
        data = True
        while data:
            data = sock.recv(recv_buffer)
            buffer += data

        while buffer.find(delim) != -1:
            line, buffer = buffer.split('\n', 1)
            yield line
        return


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
            time.sleep(0.01)


def callback(data):
    return

#rospy.Subscriber('drive_parameters', drive_param, callback)

newthread = ServerThread(TCP_IP, TCP_PORT_JAVA_PYTH, BUFFER_SIZE)
newthread.daemon = True
newthread.start()

#newthread2 = ClientThread(TCP_IP, TCP_PORT_PYTH_JAVA, BUFFER_SIZE)
#newthread2.daemon = True
#newthread2.start()

rospy.spin()

print "newthread2 finished"
