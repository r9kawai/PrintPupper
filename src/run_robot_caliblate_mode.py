import numpy as np
import re
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
        self.state = State()

        self.mode_btn_que = queue.Queue()
        self.subloop_exit = False
        self.subloop_thread = threading.Thread(target=self.subloop)
        self.subloop_thread.start()

        self.btn_move_plus = False
        self.btn_move_minus = False
        self.btn_test = False
        self.btn_next_servo = False
        self.btn_exit = False

        self.hardwareif.servo_params.neutral_angle_degrees = np.array(
            [[  0.,  0.,  0.,  0.],
            [ 45., 45., 45., 45.],
            [-45.,-45.,-45.,-45.]])
        self.neutraldegs = self.hardwareif.servo_params.neutral_angle_degrees
        self.offsets = np.zeros((3, 4), dtype=float)
        print('Before Calibrattion')
        print(self.neutraldegs)

        while True:
            try:
                btn = self.mode_btn_que.get(timeout=0.1)
                if btn == BTN_NEXT:
                    break
            except queue.Empty:
                continue

        mode_exit = False
        while True:
            for leg in range(4):
                for axis in range(3):
                    motor_name = self.get_motor_name(axis, leg)
                    set_point = [0, 45, 45][axis]
                    self.neutraldegs[axis, leg] = 0
                    offset = self.offsets[axis, leg]
                    offset, mode_exit = self.step_until(motor_name, axis, leg, set_point, offset)
                    self.offsets[axis, leg] = offset

                    if axis == 1:
                        self.neutraldegs[axis, leg] = set_point - offset
                    else:
                        self.neutraldegs[axis, leg] = -(set_point + offset)
                    self.set_actuator([0, 45, -45][axis], axis, leg)

                    if mode_exit:
                        break
                if mode_exit:
                    break
            if mode_exit:
                break

        print('After Calibrattion')
        print(self.neutraldegs)
        self.overwrite_ServoCalibration_file()

        self.subloop_exit = True
        self.subloop_thread.join(timeout=1)
        return


    def subloop(self):
        self.joystickif.get_command(self.state)
        pre_msg = self.joystickif.get_last_msg()
        tick = 0
        tack = True
        self.hardwareif.set_led_green(0)
        self.hardwareif.set_led_blue(0)

        while not self.subloop_exit:
            command = self.joystickif.get_command(self.state)
            msg = self.joystickif.get_last_msg()

            if (pre_msg["dpady"] == 0) and (msg["dpady"] == 1):
                self.mode_btn_que.put(BTN_MOVE_PLUS)

            if (pre_msg["dpady"] == 0) and (msg["dpady"] == -1):
                self.mode_btn_que.put(BTN_MOVE_MINUS)

            if (not pre_msg["circle"]) and (msg["circle"]):
                self.mode_btn_que.put(BTN_NEXT)

            if (not pre_msg["triangle"]) and (msg["triangle"]):
                self.mode_btn_que.put(BTN_TEST)

            if command.caliblate_mode_event:
                command.caliblate_mode_event = False
                self.mode_btn_que.put(BTN_EXIT)

            if (tick % 100) == 0:
                if tack:
                    self.hardwareif.set_led_green(1)
                    self.hardwareif.set_led_blue(0)
                else:
                    self.hardwareif.set_led_green(0)
                    self.hardwareif.set_led_blue(1)
                tack = not tack

            tick += 1
            pre_msg = msg
            time.sleep(self.configif.dt_sleep)

        self.hardwareif.set_led_green(0)
        self.hardwareif.set_led_blue(0)
        return


    def set_actuator(self, deg, axis, leg):
        rad = deg * np.pi / 180.0
        self.hardwareif.set_actuator_position(rad, axis, leg)
        return


    def get_motor_name(self, i, j):
        motor_type = {0: "abduction", 1: "inner", 2: "outer"}
        leg_pos = {0: "front-right", 1: "front-left", 2: "back-right", 3: "back-left"}
        final_name = motor_type[i] + " " + leg_pos[j]
        return final_name


    def step_until(self, motor_name, axis, leg, set_point, offset):
        print(motor_name, 'now', offset)
        mode_exit = False

        self.set_actuator(set_point + offset + 5, axis, leg)
        time.sleep(0.25)
        self.set_actuator(set_point + offset - 5, axis, leg)
        time.sleep(0.25)
        self.set_actuator(set_point + offset, axis, leg)

        btn = 0
        while True:
            try:
                btn = self.mode_btn_que.get(timeout=0.1)
            except queue.Empty:
                continue

            if btn == BTN_TEST:
                # print('BTN_TEST')
                self.set_actuator(set_point + offset + 5, axis, leg)
                time.sleep(0.25)
                self.set_actuator(set_point + offset - 5, axis, leg)
                time.sleep(0.25)

            if btn == BTN_MOVE_PLUS:
                offset += 0.5
                # print(offset, '++', sep='')

            if btn == BTN_MOVE_MINUS:
                offset -= 0.5
                # print(offset, '--', sep='')

            if btn == BTN_NEXT:
                # print('BTN_NEXT')
                break

            if btn == BTN_EXIT:
                # print('BTN_EXIT')
                mode_exit = True
                break

            self.set_actuator(set_point + offset, axis, leg)
            btn = 0

        print(motor_name, 'set', offset, '\n')
        return offset, mode_exit


    def overwrite_ServoCalibration_file(self):
        preamble1 = "# WARNING : This file is machine generated by run_robot_caliblate_mode.py."
        preamble2 = "# Edit at your own risk."
        preamble3 = "import numpy as np"
        formatted_str = [[x for x in row] for row in self.neutraldegs]

        # Overwrite ServoCalibration.py file with modified values
        with open("/home/pi/PrintPupper/src/ServoCalibration.py", "w") as f:
            print(preamble1, file = f)
            print(preamble2, file = f)
            print(preamble3, file = f)
            print("NEUTRAL_ANGLE_DEGREES = np.array(", file = f)
            print(formatted_str, file = f)
            print(")", file = f)


if __name__ == '__main__':
    config = Configuration()
    hardware_interface = HardwareInterface()
    joystick_interface = JoystickInterface(config)
    run_robot_caliblate_mode(config, hardware_interface, joystick_interface)

