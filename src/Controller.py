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

        self.hands_tick = int(0)
        self.hands_ticks = int(0)

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        self.activate_transition_mapping =  {RState.DEACT:RState.REST,  RState.REST:RState.DEACT, RState.TROT:RState.TROT, RState.HANDS:RState.HANDS}
        self.trot_transition_mapping =      {RState.DEACT:RState.DEACT, RState.REST:RState.TROT,  RState.TROT:RState.REST, RState.HANDS:RState.HANDS}
        self.hands_transition_mapping =     {RState.DEACT:RState.DEACT, RState.REST:RState.HANDS, RState.TROT:RState.TROT, RState.HANDS:RState.REST}

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
        state.pre_state = state.now_state
        if command.activate_event:
            state.now_state = self.activate_transition_mapping[state.now_state]
        elif command.trot_event:
            state.now_state = self.trot_transition_mapping[state.now_state]
        elif command.hands_event:
            state.now_state = self.hands_transition_mapping[state.now_state]

        if state.now_state == RState.TROT:
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

        # 「お手」 HANDS 機能の追加 -------------------------
        elif state.now_state == RState.HANDS:
            # 右スティック操作中は HANDS を拒否する判断処理
            if (state.pre_state == RState.REST and
                    (abs(command.pitch) >= self.config.pitch_deadband
                    or abs(command.yaw_rate) >= self.config.pitch_deadband)):
                state.now_state = RState.REST
                self.state_hands_proc(False, state, command)
            else:
                self.state_hands_proc(True, state, command)

        elif state.now_state == RState.REST:
            self.state_hands_proc(False, state, command)
            '''
            new_foot_locations = self.set_pose_to_rest(state, command)
            state.joint_angles = self.inverse_kinematics(new_foot_locations, self.config)
            '''
        # -------------------------------------------------

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

    def state_hands_proc(self, on_off, state, command):
        """
        「お手」 HANDS 機能の追加
        Args:
            state (_type_): _description_
            command (_type_): _description_
        """
        hands_ticks = int(self.config.hands_time / self.config.dt)
        hands_opy = command.pitch
        hands_opx = command.yaw_rate
        
        if on_off:
            command.pitch = 0
            command.yaw_rate = 0
        else:
            hands_opy = float(0)
            hands_opx = float(0)

        if on_off and state.pre_state == RState.REST:
            # REST -> HANDS の状態変化が起きた瞬間に hands_tick を開始する
            # (HANDS用の時間経過開始)
            self.hands_tick = int(0)

            # HANDS 目的座標への tick数と 差分行列 _dt (4足全ての stance_dt と、1足だけの shake_dt) を得る
            self.hands_stance_dt = self.config.hands_stance_ftlo / hands_ticks
            self.hands_shake_dt = self.config.hands_shake_ftlo_slice / hands_ticks
            self.hands_pitch_dt = self.config.hands_pitch / hands_ticks

        # 操作するのは右前脚FRか左前脚FLか？
        hands_FR0_FL1 = 1

        # hands_pitch 適用の処理
        if self.hands_tick != 0:
            target_pitch = self.hands_pitch_dt * self.hands_tick
            print(f"hands_opx {hands_opx:+07.2f}, hands_opy {hands_opy:+07.2f}, target_pitch {target_pitch:+07.2f}, state.pitch {state.pitch:+07.2f}, command.pitch {command.pitch:+07.2f}")

        # REST 状態(HANDS実装前)と同じポーズ生成処理をする
        # 標準姿勢 +command指示 Yaw,Pitch,Roll,Height 姿勢制御を計算した後の foot_locations行列を得る
        # foot_locations行列 = [X Y Z] [FR FL BL BR]
        foot_locations = self.set_pose_to_rest(state, command)

        if self.hands_tick != 0:
            # foot_location行列に 差分行列 stance_dt と stance_dt を差し込む
            foot_locations += (self.hands_stance_dt * self.hands_tick)
            foot_locations[:, hands_FR0_FL1] += (self.hands_shake_dt * self.hands_tick)

        # 逆運動学計算し joint_angled 行列(12自由度)を得る
        state.joint_angles = self.inverse_kinematics(foot_locations, self.config)

        if on_off:
            # HANDS 状態では ticks に向けて tick を進める
            self.hands_tick += 1
            self.hands_tick = min(self.hands_tick, hands_ticks - 1)
        else:
            # REST 状態では 0 に向けて tick を戻す（逆転動作をさせる）
            self.hands_tick -= 1
            self.hands_tick = max(self.hands_tick, 0)
        return
