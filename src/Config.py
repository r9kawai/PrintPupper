import numpy as np
from ServoCalibration import NEUTRAL_ANGLE_DEGREES
from enum import Enum

# TODO: put these somewhere else
class PWMParams:
    def __init__(self):

# StanfordQuadruped pin assigen
#       self.pins = np.array([[2, 14, 18, 23], [3, 15, 27, 24], [4, 17, 22, 25]])
#       self.range = 4000
#       self.freq = 250

# PrintPupper pin assigen
#                               FR  FL  BR  BL
        self.pins = np.array([[ 23, 17, 16,  5], \
                              [ 24, 27, 20,  6], \
                              [ 25, 22, 21, 19]])

class ServoParams:
    def __init__(self):
        self.pwm_freq = 100
        self.pwm_usec_max = 2500
        self.pwm_usec_min = 500
        self.pwm_usec_neutral = 1500
        self.pwm_usec_range = int(1000000 / self.pwm_freq)
        self.pwm_usec_per_rad = (self.pwm_usec_max - self.pwm_usec_min) / np.pi

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = NEUTRAL_ANGLE_DEGREES

# StanfordQuadruped rotate invert
#       self.servo_multipliers = np.array(
#           [[1, 1, 1, 1], [-1, 1, -1, 1], [1, -1, 1, -1]]
#       )

# PrintPupper rotate invert
#                                           FR FL BR BL
        self.servo_multipliers = np.array([[ 1, 1,-1,-1], \
                                           [-1, 1,-1, 1], \
                                           [ 1,-1, 1,-1]])

    @property
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians

class Configuration:
    def __init__(self):
        ################# CONTROLLER BASE COLOR ##############
        self.ps4_deactivated_color = {"red": 255, "blue": 0, "green": 0}
        self.ps4_activated_color = {"red": 0, "blue": 0, "green": 255}
        self.ps4_torot_color = {"red": 0, "blue": 255, "green": 0}
        self.ps4_calibration_color = {"red": 255, "blue": 0, "green": 255}

        ######################## GEOMETRY ######################
        self.LEG_FB = 0.100             # front-back distance from center line to leg axis
        self.LEG_LR = 0.045             # left-right distance from center line to leg plane
        self.ABDUCTION_OFFSET = 0.045   # distance from abduction axis to leg
        self.LEG_OPENING = 0.020        # distance from directly below to the open leg

        self.LEG_L1 = 0.110
        self.LEG_L2 = 0.120
        self.LEG_UNPRALLEL_L3 = 0.060
        self.LEG_UNPRALLEL_L4 = 0.060
        self.LEG_UNPRALLEL_L5 = 0.090
        self.UNPRALLEL_ofstX = -0.005
        self.UNPRALLEL_ofstY =  0.025

        #################### STANCE ####################
        self.delta_x = self.LEG_FB
        self.delta_y = self.LEG_LR + self.ABDUCTION_OFFSET + self.LEG_OPENING
        self.x_shift_front = 0.010
        self.x_shift_back =  0.000
        self.default_z_ref = -0.165
        self.min_z_ref = self.default_z_ref
        self.max_z_ref = self.default_z_ref + 0.050
        self.z_delta_as_down_speed = 0.020
        self.z_delta_as_down_speed_rate = 0.4

        #################### COMMANDS ####################
        self.max_x_velocity = 0.30
        self.max_x_velocity_minus = 0.12
        self.max_y_velocity = 0.12
        self.max_yaw_rate = 1.0
        self.max_pitch = 25 * np.pi / 180.0
        self.max_pitch_as_trot = 7 * np.pi / 180.0

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.01  # maximum speed [m/s]
        self.pitch_deadband = 0.06
        self.pitch_gain = 0.0
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.1
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.5
        self.max_stance_yaw = 0.2
        self.max_stance_yaw_rate = self.max_yaw_rate

        #################### GAIT #######################
        self.dt = 0.01
        self.dt_min_sleep = 0.002
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.14   # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.22  # duration of the phase when only two feet are on the ground 
        )

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.050
        self.alpha = (
            0.55  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.45  # Ratio between touchdown distance and total horizontal stance movement
        )

        self.LEG_ORIGINS = np.array(
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.x_shift_front,
                    self.delta_x + self.x_shift_front,
                    -self.delta_x - self.x_shift_back,
                    -self.delta_x - self.x_shift_back,
                ],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks
   
