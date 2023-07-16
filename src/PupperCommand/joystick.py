from socket import socket, AF_INET, SOCK_STREAM
import select
import pickle
from PS4Joystick import Joystick
import time
## you need to git clone the PS4Joystick repo and run `sudo bash install.sh`

## Configurable ##
MESSAGE_RATE = 20
PUPPER_COLOR = {"red":0, "blue":0, "green":255}

joystick = Joystick()
joystick.led_color(**PUPPER_COLOR)

JOYSOCK_HOST = 'localhost'
JOYSOCK_PORT = 51000
JOYSOCK_MAX_DATA = 2048
joysock = socket(AF_INET, SOCK_STREAM)
joysock.bind((JOYSOCK_HOST, JOYSOCK_PORT))
joysock.listen()
joysock_connect = False
print('joystick socket listen', JOYSOCK_PORT)

while True:
#   print("running")
    values = joystick.get_input()

    left_y = -values["left_analog_y"]
    right_y = -values["right_analog_y"]
    right_x = values["right_analog_x"]
    left_x = values["left_analog_x"]

    L2 = values["l2_analog"]
    R2 = values["r2_analog"]

    R1 = values["button_r1"]
    L1 = values["button_l1"]

    square = values["button_square"]
    x = values["button_cross"]
    circle = values["button_circle"]
    triangle = values["button_triangle"]

    dpadx = values["dpad_right"] - values["dpad_left"]
    dpady = values["dpad_up"] - values["dpad_down"]

    joymsg = {
        "ly": left_y,
        "lx": left_x,
        "rx": right_x,
        "ry": right_y,
        "L2": L2,
        "R2": R2,
        "R1": R1,
        "L1": L1,
        "dpady": dpady,
        "dpadx": dpadx,
        "x": x,
        "square": square,
        "circle": circle,
        "triangle": triangle,
        "message_rate": MESSAGE_RATE,
    }
#   print(joymsg)
    joymsg_data = pickle.dumps(joymsg)

    try:
        if not joysock_connect:
            sock_client, sock_client_addr = joysock.accept()
            joysock_connect = True
            print('joystick socket connect', JOYSOCK_PORT)
        sock_client.send(joymsg_data)

        recv_joymsg_data = False
        recv_ready, _, _ = select.select([sock_client], [], [], 0)
        if recv_ready:
            recv_joymsg_data = sock_client.recv(JOYSOCK_MAX_DATA)

    except:
        sock_client.close()
        joysock_connect = False
        print('joystick socket disconnect')

        if recv_joymsg_data:
            try:
                recv_msg = pickle.dumps(recv_joymsg_data)
                joystick.led_color(**recv_msg["ps4_color"])
            except:
                print('unknown msg from robot')

    time.sleep(1 / MESSAGE_RATE)