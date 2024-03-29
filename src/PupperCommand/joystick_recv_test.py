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

remain_data = True
while remain_data:
    try:
        recv_ready, dummy_1, dummy2 = select.select([joysock], [], [], 0)
        if recv_ready:
            trash_data = joysock.recv(JOYSOCK_MAX_DATA)
        else:
            remain_data = False
    except:
        pass

rate_counter = 0
rate_counter_start = time.perf_counter()
rate_counter_time = 0
btn_L1 = False
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
        else:
            continue
    except:
        continue

    try:
        jmsg = pickle.loads(recv_msg_data)
        # print(jmsg, round(rate_counter_time, 2), 'sec')
        print("PS4/USB:", int(jmsg["ps4_usb"]), " ", sep='', end='')
        print("rate:{:03d}".format(jmsg["message_rate"]), "    ", sep='', end='')
        print("L-X:{:5.2f}".format(jmsg["lx"]), " ", sep='', end='')
        print("L-Y:{:5.2f}".format(jmsg["ly"]), "    ", sep='', end='')
        print("R-X:{:5.2f}".format(jmsg["rx"]), " ", sep='', end='')
        print("R-Y:{:5.2f}".format(jmsg["ry"]), "    ", sep='', end='')
        print("R1:", int(jmsg["R1"]), "    ", sep='', end='')
        print("L1:", int(jmsg["L1"]), "    ", sep='', end='')
        print("PAD-X:{:02d}".format(jmsg["dpadx"]), " ", sep='', end='')
        print("PAD-Y:{:02d}".format(jmsg["dpady"]), "    ", sep='', end='')
        print("〇:", int(jmsg["circle"]), " ", sep='', end='')
        print("〇++:", int(jmsg["long_circle"]), "    ", sep='', end='')
        print("×:", int(jmsg["x"]), " ", sep='', end='')
        print("×++:", int(jmsg["long_x"]), "    ", sep='', end='')
        print("△:", int(jmsg["triangle"]), " ", sep='', end='')
        print("△++:", int(jmsg["long_triangle"]), "    ", sep='', end='')
        print("□:", int(jmsg["square"]), " ", sep='', end='')
        print("□++:", int(jmsg["long_square"]))
    except:
        print('unknown msg from joystick')
        continue

    if btn_L1 != jmsg["L1"]:
        if jmsg["L1"]:
            color = {"red":255, "blue":0, "green":255}
        else:
            color = {"red":255, "blue":0, "green":0}
        btn_L1 = jmsg["L1"]
        send_joystick_msg = {"ps4_color": color}
        send_msg = pickle.dumps(send_joystick_msg)
        joysock.send(send_msg)
        print('>', send_joystick_msg)

    time.sleep(1 / MESSAGE_RATE)

joysock.close()

