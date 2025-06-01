import sys
import os
# 自分の親ディレクトリをインポートパスに追加
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

import time
import pigpio
import RPi.GPIO as GPIO
from Config import PWMParams, ServoParams

LED_GREEN_GPIO = 0
LED_BLUE_GPIO = 1
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)
GPIO.setup(LED_BLUE_GPIO, GPIO.OUT)
GPIO.output(LED_GREEN_GPIO, True)
GPIO.output(LED_BLUE_GPIO, True)
print(f"GPIO LED_GREEN_GPIO pin {LED_GREEN_GPIO}")
print(f"GPIO LED_BLUE_GPIO  pin {LED_BLUE_GPIO}")
time.sleep(1)
GPIO.output(LED_GREEN_GPIO, False)
GPIO.output(LED_BLUE_GPIO, False)

gpio = pigpio.pi()
pwm_params = PWMParams()
servo_params = ServoParams()
freq = servo_params.pwm_freq
pwmrange = servo_params.pwm_usec_range
duty = servo_params.pwm_usec_neutral

for leg_index in range(4):
    for axis_index in range(3):
        pin = pwm_params.pins[axis_index, leg_index]
        gpio.set_PWM_frequency(pin, freq)
        gpio.set_PWM_range(pin, pwmrange)
        gpio.set_PWM_dutycycle(pin, duty)
        print(f"GPIO {pin} {freq} {pwmrange} {duty}")
        time.sleep(0.5)

