from socket import socket, AF_INET, SOCK_STREAM
import select
import pickle
import numpy as np
import time
from State import BehaviorState, State
from Command import Command
from Utilities import deadband, clipped_first_order_filter

class JoystickInterface:
    def __init__(
        self, config, # udp_port=8830, udp_publisher_port = 8840,
    ):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_activate_toggle = 0
        self.rx_ry_switch = False

        self.JOYSOCK_HOST = 'localhost'
        self.JOYSOCK_PORT = 51000
        self.JOYSOCK_MAX_DATA = 2048
        self.joysock_connect = False

    def get_command(self, state, do_print=False):
        command = Command()
        if not self.joysock_connect:
            try:
                self.joysock = socket(AF_INET, SOCK_STREAM)
                self.joysock.connect((self.JOYSOCK_HOST, self.JOYSOCK_PORT))
                self.joysock_connect = True
                self.last_msg = get_null_joymsg()
                print('robo > joystick connect.')
            except:
                print('robo > joystick socket listen...')
                time.sleep(1)
                return command

        try:
            recv_ready, dummy_1, dummy_2 = select.select([self.joysock], [], [], 0)
            if recv_ready:
                msg_data = self.joysock.recv(self.JOYSOCK_MAX_DATA)
                msg = pickle.loads(msg_data)
                self.last_msg = msg
            else:
                msg = self.last_msg

            if do_print:
               print(msg)

            if msg["long_triangle"]:
                msg["long_triangle"] = False
                self.rx_ry_switch = not self.rx_ry_switch
                print('RX/RY reverse')

            ####### Handle discrete commands ########
            # Check if requesting a state transition to trotting, or from trotting to resting
            gait_toggle = msg["R1"]
            command.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

            # Check if requesting a state transition to hopping, from trotting or resting
            hop_toggle = msg["x"]
            command.hop_event = (hop_toggle == 1 and self.previous_hop_toggle == 0)            
            
            activate_toggle = msg["L1"]
            command.activate_event = (activate_toggle == 1 and self.previous_activate_toggle == 0)

            # Update previous values for toggles and state
            self.previous_gait_toggle = gait_toggle
            self.previous_hop_toggle = hop_toggle
            self.previous_activate_toggle = activate_toggle

            ####### Handle continuous commands ########
            x_vel = msg["ly"] * self.config.max_x_velocity
            y_vel = msg["lx"] * -self.config.max_y_velocity
            command.horizontal_velocity = np.array([x_vel, y_vel])
            if self.rx_ry_switch:
                command.yaw_rate = (msg["ry"] * -1) * -self.config.max_yaw_rate
            else:
                command.yaw_rate = msg["rx"] * -self.config.max_yaw_rate

            message_rate = msg["message_rate"]
            message_dt = 1.0 / message_rate

            if self.rx_ry_switch:
                pitch = msg["rx"] * self.config.max_pitch
            else:
                pitch = (msg["ry"] * -1) * self.config.max_pitch
            deadbanded_pitch = deadband(
                pitch, self.config.pitch_deadband
            )
            pitch_rate = clipped_first_order_filter(
                state.pitch,
                deadbanded_pitch,
                self.config.max_pitch_rate,
                self.config.pitch_time_constant,
            )
            command.pitch = state.pitch + message_dt * pitch_rate

            height_movement = msg["dpady"]
            command.height = state.height - message_dt * self.config.z_speed * height_movement
            
            roll_movement = - msg["dpadx"]
            command.roll = state.roll + message_dt * self.config.roll_speed * roll_movement

            command.joy_ps4_usb = msg["ps4_usb"]

            return command

        except:
            if do_print:
                print("unknown msg from joystick")
            return Command()

    def set_color(self, color):
#       print('robo > ', color)
        try:
            joystick_msg = {"ps4_color": color}
            send_msg = pickle.dumps(joystick_msg)
            self.joysock.send(send_msg)
        except:
            pass

    def get_null_joymsg(self):
        null_joymsg = {
                "ly": 0,
                "lx": 0,
                "rx": 0,
                "ry": 0,
                "R1": False,
                "L1": False,
                "dpady": 0,
                "dpadx": 0,
                "x": False,
                "square": False,
                "circle": False,
                "triangle": False,
                "message_rate": 20,
            }
        return(null_joymsg)
