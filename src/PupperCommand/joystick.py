import os
from socket import socket, AF_INET, SOCK_STREAM
import select
import pickle
from PS4Joystick import Joystick
from USBJoystick import USBJoystick
import time

MESSAGE_RATE = 25
JOYSOCK_HOST = 'localhost'
JOYSOCK_PORT = 51000
JOYSOCK_MAX_DATA = 2048
DEVJS0_PATH = '/dev/input/js0'

class SwJoy:
    def __init__(self, ps4_usb, devpath):
        if ps4_usb:
            self.joy = Joystick()
        else:
            self.joy = USBJoystick(devpath)


def forPS4orUSBjoystick(ps4_usb, devpath):
    print('joy > socket listen...', JOYSOCK_PORT)
    joysock = socket(AF_INET, SOCK_STREAM)
    joysock.bind((JOYSOCK_HOST, JOYSOCK_PORT))
    joysock.listen()
    joysock_connect = False

    joydev_connect = False
    while True:
        if not joydev_connect:
            try:
                print('joy > dev on start...')
                joystick = SwJoy(ps4_usb, devpath)
                values = joystick.joy.get_input()
                time.sleep(0.1)
                joystick.joy.led_color(255, 0, 0)
                joydev_connect = True
                print('joy > dev on')
            except:
                joydev_connect = False
                print('joy > dev off')
                continue

        rate_counter = 0
        rate_counter_start = time.perf_counter()
        rate_counter_time = 0
        long_square_time = 0
        long_x_time = 0
        long_circle_time = 0
        long_triangle_time = 0
        while joydev_connect:
            if rate_counter >= MESSAGE_RATE:
                rate_counter_end = time.perf_counter()
                rate_counter_time = rate_counter_end - rate_counter_start
                rate_counter_start = rate_counter_end
                # print(round(rate_counter_time, 2), 'sec')
                rate_counter = 0
            else:
                rate_counter += 1

            try:
                values = joystick.joy.get_input()
            except:
                # print('joy > get err')
                joystick.joy.close()
                joydev_connect = False
                # print('joy > dev off')
                continue

            left_y = -values["left_analog_y"]
            right_y = -values["right_analog_y"]
            right_x = values["right_analog_x"]
            left_x = values["left_analog_x"]
            R1 = values["button_r1"]
            L1 = values["button_l1"]
            square = values["button_square"]
            x = values["button_cross"]
            circle = values["button_circle"]
            triangle = values["button_triangle"]
            dpadx = values["dpad_right"] - values["dpad_left"]
            dpady = values["dpad_up"] - values["dpad_down"]
            left_x = round(left_x, 2);
            left_y = round(left_y, 2);
            right_x = round(right_x, 2);
            right_y = round(right_y, 2);

            if square:
                long_square_time += 1
            else:
                long_square_time = 0
            if x:
                long_x_time += 1
            else:
                long_x_time = 0
            if circle:
                long_circle_time += 1
            else:
                long_circle_time = 0
            if triangle:
                long_triangle_time += 1
            else:
                long_triangle_time = 0

            joymsg = {
                "ly": left_y,
                "lx": left_x,
                "rx": right_x,
                "ry": right_y,
                "R1": R1,
                "L1": L1,
                "dpady": dpady,
                "dpadx": dpadx,
                "x": x,
                "square": square,
                "circle": circle,
                "triangle": triangle,

                "long_square": (True if long_square_time == MESSAGE_RATE else False),
                "long_x": (True if long_x_time == MESSAGE_RATE else False),
                "long_circle": (True if long_circle_time == MESSAGE_RATE else False),
                "long_triangle": (True if long_triangle_time == MESSAGE_RATE else False),

                "message_rate": MESSAGE_RATE,
                "ps4_usb" : ps4_usb,
            }
            # print(joymsg)
            joymsg_data = pickle.dumps(joymsg)

            try:
                if not joysock_connect:
                    sock_client, sock_client_addr = joysock.accept()
                    joysock_connect = True
                    print('< joystick socket connect', JOYSOCK_PORT)
                else:
                    sock_client.send(joymsg_data)

                    recv_joymsg_data = False
                    recv_ready, _, _ = select.select([sock_client], [], [], 0)
                    if recv_ready:
                        recv_joymsg_data = sock_client.recv(JOYSOCK_MAX_DATA)
                        if len(recv_joymsg_data) > 1:
                            try:
                                recv_msg = pickle.loads(recv_joymsg_data)
                                color_msg = recv_msg["ps4_color"]
                                r, b, g = tuple(color_msg.values())
                                joystick.joy.led_color(r, g, b)
                                # print('> joyled RGB', r, g, b)
                            except:
                                print('< unknown joymsg from robot')

            except:
                sock_client.close()
                joysock_connect = False
                print('< joystick socket disconnect')

            time.sleep(1 / MESSAGE_RATE)


if __name__ == '__main__':
    ps4_usb = False
    if os.path.exists(DEVJS0_PATH):
        print('joystick.py forUSBjoystick')
    else:
        print('joystick.py forPS4joystick')
        ps4_usb = True

    forPS4orUSBjoystick(ps4_usb, DEVJS0_PATH)


