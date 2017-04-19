#!/usr/bin/env python

# import socket
#
#
# TCP_IP = '127.0.0.1'
# TCP_PORT = 5005
# BUFFER_SIZE = 1024
# MESSAGE = "Hello, World!"
#
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((TCP_IP, TCP_PORT))
# s.send(MESSAGE)
# data = s.recv(BUFFER_SIZE)
# s.close()
#
# print "received data:", data

import socket
from threading import Thread
import time
from SocketServer import ThreadingMixIn


# Multithreaded Python server : TCP Server Socket Thread Pool

class ServerThread(Thread):
    def __init__(self, _IP_, _PORT_, _BUFFER_):
        print "Initializing serverthread"
        Thread.__init__(self)
        self._IP_ = _IP_
        self._PORT_ = _PORT_
        self._BUFFER_ = _BUFFER_

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self._IP_, self._PORT_))

    def run(self):
        print "Starting serverthread"
        self.server_socket.listen(4)
        (conn, (ip, port)) = self.server_socket.accept()
        print "[+] New server socket thread started for " + self._IP_ + ":" + str(self._PORT_)
        while True:
            data = conn.recv(2048)
            print "Server received data:", data


class ClientThread(Thread):
    def __init__(self, _IP_, _PORT_, _BUFFER_):
        print "Initializing clientthread"
        Thread.__init__(self)
        self._IP_ = _IP_
        self._PORT_ = _PORT_
        self._BUFFER_ = _BUFFER_

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print "Clientthread initialized"

    def run(self):
        print "Starting clientthread"
        self.client_socket.connect((self._IP_, self._PORT_))
        print "[+] New client socket thread started for " + self._IP_ + ":" + str(self._PORT_)

        i = 0
        while i < 10:
            MESSAGE = str(i) +
            self.client_socket.send(MESSAGE)
            i += 1
            time.sleep(1)


# Multithreaded Python server : TCP Server Socket Program Stub
TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024  # Usually 1024, but we need quick response

newthread = ServerThread(TCP_IP, TCP_PORT, BUFFER_SIZE)
newthread.daemon = True
newthread.start()

time.sleep(1)

newthread2 = ClientThread(TCP_IP, TCP_PORT, BUFFER_SIZE)
newthread2.daemon = True
newthread2.start()
newthread2.join()
print "newthread2 finished"

while True:
    time.sleep(1)
