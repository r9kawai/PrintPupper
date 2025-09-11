from Gaits import GaitController
from StanceController import StanceController
from SwingLegController import SwingController
from Utilities import clipped_first_order_filter
from State import RState, State

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat


class Controller:
    """Controller and planner object
    """

    def __init__(
        self,
        config,
        inverse_kinematics,
    ):
        self.config = config

        self.smoothed_yaw = 0.0  # for REST mode only
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        self.hands_transition_mapping = {RState.REST: RState.HANDS, RState.HANDS: RState.HANDS_DONE, RState.HANDS_DONE: RState.REST, RState.TROT: RState.HANDS}
        self.trot_transition_mapping = {RState.REST: RState.TROT, RState.TROT: RState.REST, RState.HANDS: RState.TROT, RState.HANDS_DONE: RState.TROT}
        self.activate_transition_mapping = {RState.DEACTIVATED: RState.REST, RState.REST: RState.DEACTIVATED}


    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep

        Returns
        -------
        Numpy array (3, 4)
            Matrix of new foot locations.
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes


    def run(self, state, command):
        """Steps the controller forward one timestep

        Parameters
        ----------
        controller : Controller
            Robot controller object.
        """

        ########## Update operating state based on command ######
        if command.activate_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.hands_event:
            state.behavior_state = self.hands_transition_mapping[state.behavior_state]

        if state.behavior_state == RState.TROT:
            state.foot_locations, contact_modes = self.step_gait(
                state,
                command,
            )

            # Apply the desired body rotation
            trot_rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            (roll, pitch, yaw) = quat2euler(state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            trot_rotated_foot_locations = rmat.T @ trot_rotated_foot_locations

            state.joint_angles = self.inverse_kinematics(
                trot_rotated_foot_locations, self.config
            )

        elif state.behavior_state == RState.HANDS:
            new_foot_locations = self.set_pose_to_rest(state, command)
            new_foot_locations[0][0] += 0.120
            new_foot_locations[2][0] += 0.060
            state.joint_angles = self.inverse_kinematics(new_foot_locations, self.config)
            '''
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, -0.09])[:, np.newaxis]
            )
            state.joint_angles = self.inverse_kinematics(
                state.foot_locations, self.config
            )
            '''

        elif state.behavior_state == RState.HANDS_DONE:
            new_foot_locations = self.set_pose_to_rest(state, command)
            new_foot_locations[0][0] += 0.090
            new_foot_locations[2][0] += 0.030
            state.joint_angles = self.inverse_kinematics(new_foot_locations, self.config)
            '''
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, -0.22])[:, np.newaxis]
            )
            state.joint_angles = self.inverse_kinematics(
                state.foot_locations, self.config
            )
            '''

        elif state.behavior_state == RState.REST:
            new_foot_locations = self.set_pose_to_rest(state, command)
            state.joint_angles = self.inverse_kinematics(new_foot_locations, self.config)

        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height
        return

    def set_pose_to_rest(self, state, command):
        yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
        self.smoothed_yaw += (
            self.config.dt
            * clipped_first_order_filter(
                self.smoothed_yaw,
                yaw_proportion * -self.config.max_stance_yaw,
                self.config.max_stance_yaw_rate,
                self.config.yaw_time_constant,
            )
        )
        # Set the foot locations to the default stance plus the standard height
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, command.height])[:, np.newaxis]
        )
        # Apply the desired body rotation
        rest_rotated_foot_locations = (
            euler2mat(
                command.roll,
                command.pitch,
                self.smoothed_yaw,
            )
            @ state.foot_locations
        )
        return rest_rotated_foot_locations

    def set_pose_to_default(self, state):
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        state.joint_angles = self.inverse_kinematics(
            state.foot_locations, self.config
        )
