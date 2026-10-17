from commands2 import Command, Subsystem
from phoenix6 import swerve, configs, utils
from typing import Callable
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from wpilib import DriverStation, Preferences
from wpimath.geometry import Rotation2d
from vision.limelight import Limelight
from generated.tuner_constants import train_constants, TunerSwerveDrivetrain

class drivetrain_subsystem(Subsystem, TunerSwerveDrivetrain):
    def __init__(self):
        Subsystem.__init__(self)
        TunerSwerveDrivetrain.__init__(self, train_constants.drivetrain_constants, [train_constants.front_left, train_constants.front_right, train_constants.back_left, train_constants.back_right])

        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()
        self._has_applied_operator_perspective = False

        self._limelight = Limelight(train_constants.limelight_name)

        Preferences.initDouble("max_angular_rate_rotations", train_constants.max_angular_rate_rotations)
        Preferences.initDouble("max_speed_percentage", train_constants.max_speed_percentage)
        Preferences.initDouble("steer_kP", train_constants.steer_kp)
        Preferences.initDouble("steer_kD", train_constants.steer_kd)
        Preferences.initDouble("drive_kP", train_constants.drive_kp)
        Preferences.initDouble("fl_encoder_offset", train_constants._front_left_encoder_offset)
        Preferences.initDouble("fr_encoder_offset", train_constants._front_right_encoder_offset)
        Preferences.initDouble("bl_encoder_offset", train_constants._back_left_encoder_offset)
        Preferences.initDouble("br_encoder_offset", train_constants._back_right_encoder_offset)

        self._last_tuning = None

        self._configure_auto_builder()

    def add_vision_measurement(self, pose, timestamp_seconds, std_devs=None):
        TunerSwerveDrivetrain.add_vision_measurement(
            self, pose, utils.fpga_to_current_time(timestamp_seconds), std_devs
        )

    def apply_request(self, request: Callable[[], swerve.requests.SwerveRequest]) -> Command:
        return self.run(lambda: self.set_control(request()))

    def _configure_auto_builder(self):
        try:
            config = RobotConfig.fromGUISettings()
        except Exception as e:
            print(f"[PathPlanner] ERROR al configurar AutoBuilder: {e}")
            return

        AutoBuilder.configure(
            lambda: self.get_state().pose,
            self.reset_pose,
            lambda: self.get_state().speeds,
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                PIDConstants(train_constants.auto_translation_kp, 0.0, 0.0),
                PIDConstants(train_constants.auto_rotation_kp, 0.0, 0.0),
            ),
            config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            self,
        )

    def periodic(self):
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color != None:
                self.set_operator_perspective_forward(
                    Rotation2d.fromDegrees(train_constants.red_alliance_perspective_deg)
                    if alliance_color == DriverStation.Alliance.kRed
                    else Rotation2d.fromDegrees(train_constants.blue_alliance_perspective_deg)
                )
                self._has_applied_operator_perspective = True
        self._apply_tuning_if_changed()
        self._update_vision()

    def _apply_tuning_if_changed(self):
        current = (
            Preferences.getDouble("max_angular_rate_rotations", train_constants.max_angular_rate_rotations),
            Preferences.getDouble("max_speed_percentage", train_constants.max_speed_percentage),
            Preferences.getDouble("steer_kP", train_constants.steer_kp),
            Preferences.getDouble("steer_kD", train_constants.steer_kd),
            Preferences.getDouble("drive_kP", train_constants.drive_kp),
            Preferences.getDouble("fl_encoder_offset", train_constants._front_left_encoder_offset),
            Preferences.getDouble("fr_encoder_offset", train_constants._front_right_encoder_offset),
            Preferences.getDouble("bl_encoder_offset", train_constants._back_left_encoder_offset),
            Preferences.getDouble("br_encoder_offset", train_constants._back_right_encoder_offset),
        )
        if current == self._last_tuning:
            return
        self._last_tuning = current
        max_angular, max_speed, steer_kp, steer_kd, drive_kp, fl_off, fr_off, bl_off, br_off = current

        train_constants.max_angular_rate_rotations = max_angular
        train_constants.max_speed_percentage = max_speed
        steer_slot0 = configs.Slot0Configs().with_k_p(steer_kp).with_k_d(steer_kd)
        drive_slot0 = configs.Slot0Configs().with_k_p(drive_kp)
        offsets = [fl_off, fr_off, bl_off, br_off]  # mismo orden que self.modules

        for module, offset in zip(self.modules, offsets):
            module.steer_motor.configurator.apply(steer_slot0)
            module.drive_motor.configurator.apply(drive_slot0)
            module.encoder.configurator.apply(
                configs.MagnetSensorConfigs().with_magnet_offset(offset)
            )

    def _update_vision(self):
        state = self.get_state()
        self._limelight.set_robot_orientation(state.pose.rotation().degrees())

        if abs(state.speeds.omega) > train_constants.vision_max_angular_rate:
            return  # girando muy rápido, no confiar en la visión este ciclo

        estimate = self._limelight.get_pose_estimate()
        if estimate is None:
            return  # sin Limelight o sin tags: no pasa nada, seguimos con odometría sola

        theta_std = (
            train_constants.vision_std_dev_theta_multi_tag
            if estimate.tag_count >= 2
            else train_constants.vision_std_dev_theta_single_tag
        )
        self.add_vision_measurement(
            estimate.pose,
            estimate.timestamp_seconds,
            (train_constants.vision_std_dev_xy, train_constants.vision_std_dev_xy, theta_std),
        )