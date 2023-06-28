from socket import socket, AF_INET, SOCK_STREAM
import pickle

JOYSOCK_HOST = 'localhost'
JOYSOCK_PORT = 51000
JOYSOCK_MAX_DATA = 2048
sock = socket(AF_INET, SOCK_STREAM)
sock.connect((JOYSOCK_HOST, JOYSOCK_PORT))

while True:
    msg_data = sock.recv(JOYSOCK_MAX_DATA)
    try:
        msg = pickle.loads(msg_data)
        print(msg)
    except:
        print('unknown msg from joystick')

