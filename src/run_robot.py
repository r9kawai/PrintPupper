import sys
import numpy as np
import time
from Controller import Controller
from JoystickInterface import JoystickInterface
from State import State
from HardwareInterface import HardwareInterface
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics
from State import RState, State
from run_robot_caliblate_mode_2 import run_robot_caliblate_mode

def main(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface(config)

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    print("Creating joystick listener...", end="")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    # Wait until the activate button has been pressed
    joystick_interface.set_color(config.ps4_deactivated_color)
    last_loop_time = time.time()
    while True:
        wait_loop = 0
        led_blink = 0

        print("Robot Waiting L1 button to activate robot")
        wait_loop_first = False
        while True:
            if (wait_loop % (25 if wait_loop_first else 100)) == 0:
                hardware_interface.set_led_green(bool(led_blink % 2))
                led_blink += 1
            wait_loop +=1
            command = joystick_interface.get_command(state)
            # PS4 gamepad from bluetooth mode as slow blink / USB gamepad as first blink
            if command.joy_ps4_usb:
                wait_loop_first = False
            else:
                wait_loop_first = True
            if command.activate_event == 1:
                break
            if command.caliblate_mode_event:
                command.caliblate_mode_event = False
                joystick_interface.set_color(config.ps4_calibration_color)
                run_robot_caliblate_mode(config, hardware_interface, joystick_interface)
                joystick_interface.set_color(config.ps4_deactivated_color)
                sys.exit()
            time.sleep(0.01)

        print("Robot Activate")
        hardware_interface.set_led_green(True)
        joystick_interface.set_color(config.ps4_activated_color)
        while True:
            d_time = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)

            # Update PS4 Joystick LED, Update Onboard LED, Exit(to Deactivate) check
            if update_leds_loop_exit(config, state, command, hardware_interface, joystick_interface):
                break

            # Step the controller forward by dt
            controller.run(state, command)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_positions(state.joint_angles)

            # cycle tune
            t_time = time.time()
            dt = (float)(t_time - d_time)
            dt_dt = (float)(config.dt - dt)
            dt_dt = round(dt_dt, 3)
            if (dt_dt > 0) and (dt_dt >= config.dt_min_sleep):
                time.sleep(dt_dt)

def update_leds_loop_exit(config, state, command, hardware_interface, joystick_interface):
    if command.activate_event:
        if state.behavior_state == RState.REST:
            print("Robot Deactivate")
            joystick_interface.set_color(config.ps4_deactivated_color)
            hardware_interface.set_led_blue(False)
            return True

    if command.trot_event:
        if state.behavior_state == RState.REST:
            joystick_interface.set_color(config.ps4_torot_color)
            hardware_interface.set_led_blue(True)
            print("Robot start Torot")
        else:
            joystick_interface.set_color(config.ps4_activated_color)
            hardware_interface.set_led_blue(False)
            print("Robot stop Torot")

    return False

if __name__ == "__main__":
    main()
