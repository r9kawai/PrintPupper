import numpy as np
from Config import Configuration

class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        _config = Configuration()
        self.height = _config.default_z_ref
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.joy_ps4_usb = True

        self.hands_event = False
        self.hands_event_arg_RL = int(0)
        self.hands_event_arg_PUSH = int(0)
        self.trot_event = False
        self.activate_event = False
        self.caliblate_mode_event = False
