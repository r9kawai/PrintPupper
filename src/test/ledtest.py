import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(0, GPIO.OUT)
GPIO.setup(1, GPIO.OUT)
try:
    while True:
        GPIO.output(0, True)
        GPIO.output(1, False)
        time.sleep(0.5)
        GPIO.output(0, False)
        GPIO.output(1, True)
        time.sleep(0.5)

except KeyboardInterrupt:
    pass

GPIO.cleanup()
