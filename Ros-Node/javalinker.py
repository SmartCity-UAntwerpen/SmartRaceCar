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
from SocketServer import ThreadingMixIn
# Multithreaded Python server : TCP Server Socket Thread Pool
# from tcpserver import conn


class ClientThread(Thread):
    def __init__(self, ip, port, conn):
        Thread.__init__(self)
        self.ip = ip
        self.port = port
        self.conn = conn
        print "[+] New server socket thread started for " + ip + ":" + str(port)

    def run(self):
        while True:
            data = self.conn.recv(2048)
            print "Server received data:", data
            MESSAGE = raw_input("Multithreaded Python server : Enter Response from Server/Enter exit:")
            if MESSAGE == 'exit':
                break
            self.conn.send(MESSAGE)  # echo

# Multithreaded Python server : TCP Server Socket Program Stub
TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024  # Usually 1024, but we need quick response
tcpServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcpServer.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tcpServer.bind((TCP_IP, TCP_PORT))


def get_thread_server():
    tcpServer.listen(4)
    print "Multithreaded Python server : Waiting for connections from TCP clients..."
    print "pipi"
    (conn, (ip, port)) = tcpServer.accept()
    print "pis"
    newthread = ClientThread(ip, port, conn)
    print "kak"
    newthread.start()
    print "stront"
    # return newthread

server = get_thread_server()
server.start()

helloworld = "Hello, World!"
print helloworld
tcpServer.send(helloworld)
answer = tcpServer.recv(BUFFER_SIZE)
print "I received data: ", answer

server.join()