from socket import socket, AF_INET, SOCK_STREAM
import select
import pickle
import time

MESSAGE_RATE = 20
JOYSOCK_HOST = 'localhost'
JOYSOCK_PORT = 51000
JOYSOCK_MAX_DATA = 2048
joysock_connect = False
while not joysock_connect:
    try:
        joysock = socket(AF_INET, SOCK_STREAM)
        joysock.connect((JOYSOCK_HOST, JOYSOCK_PORT))
        joysock_connect = True
    except:
        print('no joystick socket listen')
    time.sleep(1)

rate_counter = 0
rate_counter_start = time.perf_counter()
rate_counter_time = 0
while joysock_connect:
    if rate_counter >= MESSAGE_RATE:
        rate_counter_end = time.perf_counter()
        rate_counter_time = rate_counter_end - rate_counter_start
        rate_counter_start = rate_counter_end
        rate_counter = 0
    else:
        rate_counter += 1

    try:
        recv_msg_data = False
        recv_ready, dummy_1, dummy2 = select.select([joysock], [], [], 0)
        if recv_ready:
            recv_msg_data = joysock.recv(JOYSOCK_MAX_DATA)
    except:
        joysock.close()
        joysock_connect = False
        continue

    try:
        decode_msg = pickle.loads(recv_msg_data)
        print(decode_msg, round(rate_counter_time, 2), 'sec')
    except:
        print('unknown msg from joystick')

    if rate_counter == 0:
        #print(decode_msg['L1'])
        if decode_msg['L1']:
            color = {"red":255, "blue":0, "green":0}
        else:
            color = {"red":0, "blue":255, "green":0}
        send_joystick_msg = {"ps4_color": color}
        send_msg = pickle.dumps(send_joystick_msg)
        joysock.send(send_msg)
        #print('>', send_joystick_msg)

    time.sleep(1 / MESSAGE_RATE)

joysock.close()
