import numpy as np
import time
from IMU import IMU
from Controller import Controller
from JoystickInterface import JoystickInterface
from State import State
from HardwareInterface import HardwareInterface
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics

# PrintPupper
import RPi.GPIO as GPIO
GPIO_LED_G = 0
GPIO_LED_B = 1
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_LED_G, GPIO.OUT)
GPIO.setup(GPIO_LED_B, GPIO.OUT)
GPIO.output(GPIO_LED_G, False)
GPIO.output(GPIO_LED_B, False)

def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # Wait until the activate button has been pressed
    while True:
        # PrintPupper
        wait_loop = 0
        led_blink = False

        print("Waiting for L1 to activate robot.")
        while True:
            # PrintPupper
            if wait_loop % 5 == 0:
                if led_blink:
                    led_blink = False
                else:
                    led_blink = True
                GPIO.output(GPIO_LED_G, led_blink)
            wait_loop += 1

            command = joystick_interface.get_command(state)
            joystick_interface.set_color(config.ps4_deactivated_color)
            if command.activate_event == 1:
                break
            time.sleep(0.1)

        print("Robot activated.")
        # PrintPupper
        GPIO.output(GPIO_LED_G, True)

        joystick_interface.set_color(config.ps4_color)

        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                hardware_interface.deactivate()
                break

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(state.joint_angles)


main()
