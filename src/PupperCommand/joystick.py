from socket import socket, AF_INET, SOCK_STREAM
import select
import pickle
from PS4Joystick import Joystick
import time
## you need to git clone the PS4Joystick repo and run `sudo bash install.sh`

## Configurable ##
MESSAGE_RATE = 25

JOYSOCK_HOST = 'localhost'
JOYSOCK_PORT = 51000
JOYSOCK_MAX_DATA = 2048
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
            joystick = Joystick()
            values = joystick.get_input()
            joydev_connect = True
            joystick.led_color(255, 0, 0)
            print('joy > dev on')
        except:
            joydev_connect = False
            print('joy > dev off')
        time.sleep(1)

    else:
        rate_counter = 0
        rate_counter_start = time.perf_counter()
        rate_counter_time = 0
        while joydev_connect:
            if rate_counter >= MESSAGE_RATE:
                rate_counter_end = time.perf_counter()
                rate_counter_time = rate_counter_end - rate_counter_start
                rate_counter_start = rate_counter_end
#               print(round(rate_counter_time, 2), 'sec')
                rate_counter = 0
            else:
                rate_counter += 1

            try:
                values = joystick.get_input()
            except:
                print('joy > get err')
                joystick.close()
                joydev_connect = False
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
        
            left_x = round(left_x, 3);
            left_y = round(left_y, 3);
            right_x = round(right_x, 3);
            right_y = round(right_y, 3);
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
                    try:
                        recv_msg = pickle.loads(recv_joymsg_data)
                        color_msg = recv_msg["ps4_color"]
                        r, b, g = tuple(color_msg.values())
                        joystick.led_color(r, g, b)
        #               print('> joyled RGB', r, g, b)
                    except:
                        print('unknown msg from robot')
        
            except:
                sock_client.close()
                joysock_connect = False
                print('joystick socket disconnect')
                joystick.close()
                joydev_connect = False
        
            time.sleep(1 / MESSAGE_RATE)
