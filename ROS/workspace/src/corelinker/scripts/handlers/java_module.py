from location import Location

import socket
import json
import time
from threading import Thread

TCP_IP = '127.0.0.1'
TCP_PORT_JAVA_PYTH = 5005
TCP_PORT_PYTH_JAVA = 5006
BUFFER_SIZE = 64


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


def send_message(json_string):
    global TCP_IP, TCP_PORT_PYTH_JAVA, connected

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False
    while not connected:
        try:
            client_socket.connect((TCP_IP, TCP_PORT_PYTH_JAVA))
            connected = True
        except socket.error, exc:
            logger.log_warning("[CLIENTSOCKET] Cannot send data:  " + json_string + "   Trying again." + str(exc))
            connected = False
            time.sleep(1)
    logger.log_debug("[CLIENTSOCKET] Socket created & connected")

    client_socket.sendall(json_string)
    logger.log_debug("[CLIENTSOCKET] data sent: " + json_string)

    client_socket.close()
    logger.log_debug("[CLIENTSOCKET] Socket closed")


def connect():
    jsonmessage = {'connect': {'x': 0, 'y': 0}}
    json_string = json.dumps(jsonmessage)
    send_message(json_string)
    logger.log_info("Connected to Core.")
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
            logger.log_debug("[SERVERSOCKET] Server received data: " + data_string)
            json_string = json.loads(data_string)

            from .. import javalinker
            javalinker.get_type(json_string)


def start_thread():
    newthread = ServerThread(TCP_IP, TCP_PORT_JAVA_PYTH, BUFFER_SIZE)
    newthread.daemon = True
    newthread.start()


def set_logger(logger_argument):
    global logger
    logger = logger_argument


