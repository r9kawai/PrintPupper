import math
import pigpio
import RPi.GPIO as GPIO
from Config import Configuration, ServoParams, PWMParams
from compute_unparallel_link_knee import compute_unparallel_link_knee

LED_GREEN_GPIO = 0
LED_BLUE_GPIO = 1

class HardwareInterface:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)
        GPIO.setup(LED_BLUE_GPIO, GPIO.OUT)
        GPIO.output(LED_GREEN_GPIO, False)
        GPIO.output(LED_BLUE_GPIO, False)
        print('GPIO LED GREEN [ ', LED_GREEN_GPIO, 'pin ]', sep='')
        print('GPIO LED BLUE  [ ', LED_BLUE_GPIO, 'pin ]', sep='')

        self.config = Configuration()
        self.pi = pigpio.pi()
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.initialize_pwm()
        self.culk = compute_unparallel_link_knee(self.config)
        return

    def set_actuator_postions(self, joint_angles):
        self.send_servo_commands(joint_angles)
        return

    def set_actuator_position(self, joint_angle, axis, leg):
        self.send_servo_command(joint_angle, axis, leg)
        return

    def deactivate(self):
        self.deactivate_servos()
        return

    def initialize_pwm(self):
        print('GPIO ', self.servo_params.pwm_freq, 'Hz ', self.servo_params.pwm_usec_range, 'range ', self.servo_params.pwm_usec_neutral, 'neutral',  sep='')
        for leg_index in range(4):
            if leg_index == 0:
                print('GPIO FR-[ ', end='')
            if leg_index == 1:
                print('GPIO FL-[ ', end='')
            if leg_index == 2:
                print('GPIO BR-[ ', end='')
            if leg_index == 3:
                print('GPIO BL-[ ', end='')
            for axis_index in range(3):
                self.pi.set_PWM_frequency(
                    self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_freq
                )
                self.pi.set_PWM_range(self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_usec_range)
                print('{:02d}'.format(self.pwm_params.pins[axis_index, leg_index]), 'pin ', sep='', end='')
            print(']')
        return

    def deactivate_servos(self):
        for leg_index in range(4):
            for axis_index in range(3):
                self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], 0)
        return

    def angle_to_pwmdutycycle(self, angle, axis_index, leg_index):
        neutral = self.servo_params.neutral_angles[axis_index, leg_index]
        multi = self.servo_params.servo_multipliers[axis_index, leg_index]
        angle_deviation = (angle - neutral) * multi
        usec_val = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle_deviation)
        usec_val_limited = max(self.servo_params.pwm_usec_min, min(usec_val, self.servo_params.pwm_usec_max))
        return int(usec_val_limited)

    def angle_to_duty_cycle(self, angle, axis_index, leg_index):
        pulsewidth_micros = self.angle_to_pwm(angle, axis_index, leg_index)
        duty_cyle = int(pulsewidth_micros / 1e6 * self.servo_params.pwm_freq * self.servo_params.pwm_usec_range)
        return duty_cyle

    def angle_to_pwm(self, angle, axis_index, leg_index):
        neutral = self.servo_params.neutral_angles[axis_index, leg_index]
        multi = self.servo_params.servo_multipliers[axis_index, leg_index]
        angle_deviation = (angle - neutral) * multi
        pulse_width_micros = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle_deviation)
        return pulse_width_micros

    def send_servo_command(self, joint_angle, axis, leg):
#       duty_cycle = self.angle_to_duty_cycle(joint_angle, axis, leg)
        duty_cycle = self.angle_to_pwmdutycycle(joint_angle, axis, leg)
        self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis, leg], duty_cycle)
        return

    def send_servo_commands(self, joint_angles):
        debug = False

        for leg_index in range(4):
            # 非平行リンク機構の導入による改修 ------------------------------------------------------------------------
            # amend for PrintPupper v0.2's unparallel link mechanism
            rad0 = joint_angles[0, leg_index] * self.servo_params.servo_multipliers[0, leg_index]
            rad1 = joint_angles[1, leg_index] * self.servo_params.servo_multipliers[1, leg_index]
            rad2 = joint_angles[2, leg_index] * self.servo_params.servo_multipliers[2, leg_index]
            coxa  = rad0
            knee  = rad2

            # compute_unparallel_link_knee の表現型に変換してから非平行リンクを介した場合の knee サーボ角度を再計算
            if leg_index == 0 or leg_index == 2:
                leg = (math.pi / 2) - rad1
                leg = -leg
                mirror = True
            else:
                leg = (math.pi / 2) + rad1
                mirror = False
            kneeX = self.culk.compute(leg, knee, mirror)

            if math.isnan(kneeX):
                debug = True
                print("compute Err!")

            if debug:
                print(f"LEG{leg_index} : coxa {coxa:+07.2f}({math.degrees(coxa):+07.2f}), ", end="")
                print(f"leg {leg:+07.2f}({math.degrees(leg):+07.2f}), ", end="")
                print(f"knee {knee:+07.2f}({math.degrees(knee):+07.2f}), ", end="")
                print(f"kneeX {kneeX:+07.2f}({math.degrees(kneeX):+07.2f})")
                if leg_index == 3:
                        print("")

            # 再計算結果 knee 角度指示に戻す
            joint_angles[2, leg_index] = kneeX * self.servo_params.servo_multipliers[2, leg_index]
            # ---------------------------------------------------------------------------------------------------------

            for axis_index in range(3):
                angle = joint_angles[axis_index, leg_index]
                duty_cycle = self.angle_to_pwmdutycycle(angle, axis_index, leg_index)
                self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], duty_cycle)
        return

    def send_servo_commands_dbg(self, joint_angles):
        for leg_index in range(4):
            if leg_index == 0:
                print('FR-[ ', end='')
            if leg_index == 1:
                print('FL-[ ', end='')
            if leg_index == 2:
                print('BR-[ ', end='')
            if leg_index == 3:
                print('BL-[ ', end='')
            for axis_index in range(3):
                angle = joint_angles[axis_index, leg_index]
#               duty_cycle = self.angle_to_duty_cycle(angle, axis_index, leg_index)
                duty_cycle = self.angle_to_pwmdutycycle(angle, axis_index, leg_index)
                self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], duty_cycle)
                print('{:04d}'.format(duty_cycle), end=' ')
            print('] ', end='')
        print('')
        return

    def set_led_green(self, onoff):
        GPIO.output(LED_GREEN_GPIO, onoff)
        return

    def set_led_blue(self, onoff):
        GPIO.output(LED_BLUE_GPIO, onoff)
        return

    """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin
def pwm_to_duty_cycle(pulsewidth_micros, pwm_params):

    Parameters
    ----------
    pulsewidth_micros : float
        Width of the pwm signal in microseconds
    pwm_params : PWMParams
        PWMParams object

    Returns
    -------
    float
        PWM duty cycle corresponding to the pulse width
    return int(pulsewidth_micros / 1e6 * pwm_params.freq * pwm_params.range)
    """

    """Converts a desired servo angle into the corresponding PWM command
def angle_to_pwm(angle, servo_params, axis_index, leg_index):

    Parameters
    ----------
    angle : float
        Desired servo angle, relative to the vertical (z) axis
    servo_params : ServoParams
        ServoParams object
    axis_index : int
        Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
    leg_index : int
        Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

    Returns
    -------
    float
        PWM width in microseconds
    angle_deviation = (
        angle - servo_params.neutral_angles[axis_index, leg_index]
    ) * servo_params.servo_multipliers[axis_index, leg_index]
    pulse_width_micros = (
        servo_params.neutral_position_pwm
        + servo_params.micros_per_rad * angle_deviation
    )
    return pulse_width_micros
    """

