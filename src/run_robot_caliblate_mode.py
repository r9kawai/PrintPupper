import numpy as np
import time
import threading
import queue
from Config import Configuration
from HardwareInterface import HardwareInterface
from State import State
from JoystickInterface import JoystickInterface

"""
NEUTRAL_ANGLE_DEGREES = np.array(
[[  0.,  0.,  0.,  0.],
 [ 45., 45., 45., 45.],
 [-45.,-45.,-45.,-45.]]
)
"""

MESSAGE_RATE = 25
BTN_MOVE_PLUS = 1
BTN_MOVE_MINUS = 2
BTN_TEST = 3
BTN_NEXT = 4
BTN_EXIT = 5

class run_robot_caliblate_mode():
    def __init__(self, config, hardware_interface, joystick_interface):
        self.configif = config
        self.hardwareif = hardware_interface
        self.joystickif = joystick_interface

        self.subloop_exit = False
        self.subloop_thread = threading.Thread(target=self.subloop)
        self.subloop_thread.start()
        self.mode_btn_que = queue.Queue()

        self.btn_move_plus = False
        self.btn_move_minus = False
        self.btn_test = False
        self.btn_next_servo = False
        self.btn_exit = False

        self.hardwareif.servo_params.neutral_angle_degrees = np.zeros((3, 4))
        mode_exit = False
        while True:
            for leg in range(4):
                for axis in range(3):
                    motor_name = self.get_motor_name(axis, leg)
                    set_point = self.get_motor_setpoint(axis, leg)
                    self.hardwareif.servo_params.neutral_angle_degrees[axis, leg] = 0
                    self.hardwareif.set_actuator_position(self.degrees_to_radians(set_point), axis, leg)
                    offset = 0
                    print(motor_name, leg, axis, offset)
                    offset, mode_exit = self.step_until(axis, leg, set_point)
                    print(motor_name, leg, axis, offset)
                    if axis == 1:
                        self.hardwareif.servo_params.neutral_angle_degrees[axis, leg] = set_point - offset
                    else:
                        self.hardwareif.servo_params.neutral_angle_degrees[axis, leg] = -(set_point + offset)
                    hardware_interface.set_actuator_position(self.degrees_to_radians([0, 45, -45][axis]), axis, leg)
                    if mode_exit:
                        break
                if mode_exit:
                    break
            if mode_exit:
                break


        self.subloop_exit = True
        self.subloop_thread.join(timeout=1)
        return

    def subloop(self):
        joystick_interface.get_command(state)
        pre_msg = joystick_interface.get_last_msg()
        while not self.subloop_exit:
            joystick_interface.get_command(state)
            msg = joystick_interface.get_last_msg()

            if (pre_msg["dpady"] == 0) and (msg["dpady"] == 1):
                self.mode_btn_que.put(BTN_MOVE_PLUS)

            if (pre_msg["dpady"] == 0) and (msg["dpady"] == -1):
                self.mode_btn_que.put(BTN_MOVE_MINUS)

            if (not pre_msg["circle"]) and (msg["circle"]):
                self.mode_btn_que.put(BTN_NEXT)

            if (not pre_msg["triangle"]) and (msg["triangle"]):
                self.mode_btn_que.put(BTN_TEST)

            if msg["long_x"]:
                self.mode_btn_que.put(BTN_EXIT)

            pre_msg = msg
            time.sleep(1 / MESSAGE_RATE)
        return

    def degrees_to_radians(self, input_array):
        return input_array * np.pi / 180.0

    def get_motor_name(self, i, j):
        motor_type = {0: "abduction", 1: "inner", 2: "outer"}
        leg_pos = {0: "front-right", 1: "front-left", 2: "back-right", 3: "back-left"}
        final_name = motor_type[i] + " " + leg_pos[j]
        return final_name

    def get_motor_setpoint(self, i, j):
        data = np.array([[0, 0, 0, 0], [45, 45, 45, 45], [45, 45, 45, 45]])
        return data[i, j]

    def step_until(self, axis, leg, set_point):
        offset = 0
        mode_exit = False
        btn = 0
        while True:
            try:
                btn = self.mode_btn_que.get(timeout=0.2)
            except queue.Empty:
                continue

            if btn == BTN_TEST:
                print('BTN_TEST')
                self.hardwareif.set_actuator_position(self.degrees_to_radians(set_point + offset + 5), axis, leg)
                time.sleep(0.25)
                self.hardwareif.set_actuator_position(self.degrees_to_radians(set_point + offset - 5), axis, leg)
                time.sleep(0.25)

            if btn == BTN_MOVE_PLUS:
                offset += 1.0
                print(offset, '++', sep='')

            if btn == BTN_MOVE_MINUS:
                offset -= 1.0
                print(offset, '--', sep='')

            if btn == BTN_NEXT:
                print('BTN_NEXT')
                break

            if btn == BTN_EXIT:
                print('BTN_EXIT')
                mode_exit = True
                break

            self.hardwareif.set_actuator_position(self.degrees_to_radians(set_point + offset), axis, leg)
            btn = 0

        return offset, mode_exit


if __name__ == '__main__':
    config = Configuration()
    hardware_interface = HardwareInterface()
    state = State()
    joystick_interface = JoystickInterface(config)
    run_robot_caliblate_mode(config, hardware_interface, joystick_interface)

