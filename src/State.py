import numpy as np
from enum import Enum
from Config import Configuration

class State:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        _config = Configuration()
        self.height = _config.default_z_ref
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.now_state = RState.REST
        self.pre_state = RState.REST
        self.ticks = 0
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
        self.quat_orientation = np.array([1, 0, 0, 0])

class RState(Enum):
    DEACT = -1
    REST = 0
    TROT = 1
    HANDS = 2
