import numpy as np
import time
import threading
from Config import Configuration
from HardwareInterface import HardwareInterface
from JoystickInterface import JoystickInterface

"""
NEUTRAL_ANGLE_DEGREES = np.array(
[[  0.,  0.,  0.,  0.],
 [ 45., 45., 45., 45.],
 [-45.,-45.,-45.,-45.]]
)
"""

configif = None
hardwareif = None
joystickif = None
subloop_exit = False

def degrees_to_radians(input_array):
    return input_array * np.pi / 180.0

def get_motor_name(i, j):
    motor_type = {0: "abduction", 1: "inner", 2: "outer"}
    leg_pos = {0: "front-right", 1: "front-left", 2: "back-right", 3: "back-left"}
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name

def get_motor_setpoint(i, j):
    data = np.array([[0, 0, 0, 0], [45, 45, 45, 45], [45, 45, 45, 45]])
    return data[i, j]

def subloop():
    while not subloop_exit:
        time.sleep(0.1)

    return

def step_until(axis, leg, set_point):
    offset = 0

    offset += 10.0
    hardwareif.set_actuator_position(degrees_to_radians(set_point + offset), axis, leg)
    time.sleep(0.5)

    offset -= 10.0
    hardwareif.set_actuator_position(degrees_to_radians(set_point + offset), axis, leg)
    time.sleep(0.5)

    return offset

def run_robot_caliblate_mode(config, hardware_interface, joystick_interface):
    configif = config
    hardwareif = hardware_interface
    joystickif = joystick_interface

    subloop_exit = False
    subloop_thread = threading.Thread(target=subloop)
    subloop_thread.start()

    hardwareif.servo_params.neutral_angle_degrees = np.zeros((3, 4))
    for leg in range(4):
        for axis in range(3):
            motor_name = get_motor_name(axis, leg)
            set_point = get_motor_setpoint(axis, leg)
            hardwareif.servo_params.neutral_angle_degrees[axis, leg] = 0
            hardwareif.set_actuator_position(degrees_to_radians(set_point), axis, leg)
            print(motor_name, leg, axis)
            offset = step_until(axis, leg, set_point)
            if axis == 1:
                hardwareif.servo_params.neutral_angle_degrees[axis, leg] = set_point - offset
            else:
                hardwareif.servo_params.neutral_angle_degrees[axis, leg] = -(set_point + offset)
            hardware_interface.set_actuator_position(degrees_to_radians([0, 45, -45][axis]), axis, leg)

    subloop_exit = True
    subloop_thread.join(timeout=1)
    return

if __name__ == '__main__':
    config = Configuration()
    hardware_interface = HardwareInterface()
    joystick_interface = JoystickInterface(config)
    run_robot_caliblate_mode(config, hardware_interface, joystick_interface)

