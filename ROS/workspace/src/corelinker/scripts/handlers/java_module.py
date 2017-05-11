import json
import socket
import time

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
    logger.log_info("[JAVAMODULE] Connected to Core")
    global connected
    connected = True


def set_logger(logger_argument):
    global logger
    logger = logger_argument


