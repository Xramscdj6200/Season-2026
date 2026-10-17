import math

from commands2 import Command
from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.filter import Debouncer
from wpimath.units import rotationsToRadians
from wpimath.geometry import Rotation2d, Translation2d
from vision.limelight import Limelight
        
from generated.tuner_constants import train_constants

"""
|
|   Teleoperado
|
"""
class drive(Command):
    def __init__(self, drivetrain, pilot, copilot):
        super().__init__()
        self.drivetrain = drivetrain

        self.pilot = pilot
        self.copilot = copilot

        self.addRequirements(drivetrain)

        self.request_pilot = (
            swerve.requests.FieldCentric()
            .with_deadband(train_constants.speed_at_12_volts * train_constants.max_speed_percentage * train_constants.pilot_joystick_deadband)
            .with_rotational_deadband(rotationsToRadians(train_constants.max_angular_rate_rotations) * train_constants.pilot_joystick_deadband)
            .with_drive_request_type(train_constants.drive_request_type)
        )

        if self.copilot != None and self.copilot.getHID().isConnected():
            pass

    def execute(self):
        max_speed = train_constants.speed_at_12_volts * train_constants.max_speed_percentage
        self.drivetrain.set_control(
            self.request_pilot
            .with_velocity_x(-self.pilot.getLeftY() * max_speed)
            .with_velocity_y(-self.pilot.getLeftX() * max_speed)
            .with_rotational_rate(self.pilot.getRightX() * rotationsToRadians(train_constants.max_angular_rate_rotations))
        )

class idle(Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.request = swerve.requests.Idle()

    def execute(self):
        self.drivetrain.set_control(self.request)


class brake(Command):
    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.request = swerve.requests.SwerveDriveBrake()

    def execute(self):
        self.drivetrain.set_control(self.request)


class point(Command):
    def __init__(self, drivetrain, pilot):
        super().__init__()
        self.drivetrain = drivetrain
        self.pilot = pilot
        self.addRequirements(drivetrain)
        self.request = swerve.requests.PointWheelsAt()

    def execute(self):
        self.drivetrain.set_control(
            self.request.with_module_direction(
                Rotation2d(-self.pilot.getLeftY(), -self.pilot.getLeftX())
            )
        )

class seed_command(Command):
    def __init__(self, drivetrain, heading=None):
        super().__init__()
        self.drivetrain = drivetrain
        self.heading = heading
        self.addRequirements(drivetrain)

    def initialize(self):
        if self.heading is None:
            self.drivetrain.seed_field_centric()
        else:
            self.drivetrain.seed_field_centric(self.heading)

    def isFinished(self):
        return True


class aim(Command):
    def __init__(self, drivetrain, pilot, on_distance=None):
        super().__init__()
        self.drivetrain = drivetrain
        self.pilot = pilot                 # quien conduce (traslación)
        self.on_distance = on_distance     # opcional: callback(distancia_m)
        self._limelight = Limelight(train_constants.limelight_name)
        self.addRequirements(drivetrain)

        self.max_speed = (
            train_constants.speed_at_12_volts * train_constants.max_speed_percentage
        )
        self.request = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self.max_speed * train_constants.pilot_joystick_deadband)
            .with_drive_request_type(train_constants.drive_request_type)
        )
        self.request.heading_controller.setPID(
            train_constants.heading_kp,
            train_constants.heading_ki,
            train_constants.heading_kd,
        )
        self.request.heading_controller.enableContinuousInput(-math.pi, math.pi)

        self._locked = False
        self._target_heading = 0.0
        self._distance = 0.0
        self._is_red = False
        self._own_tag_ids = []

        self._active = False
        self._aimed = False
        self._aimed_debounce = Debouncer(0.15, Debouncer.DebounceType.kRising)

    # ---------- Estado para otros comandos ----------
    def ready(self):
        """True solo mientras el comando corre y el robot está alineado y estable."""
        return self._active and self._aimed

    # ---------- Ciclo ----------
    def initialize(self):
        self._is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        self._own_tag_ids = (
            train_constants.red_hub_aimable_tag_ids
            if self._is_red
            else train_constants.blue_hub_aimable_tag_ids
        )
        self._limelight.set_fiducial_id_filter(self._own_tag_ids)
        self._locked = False
        self._aimed = False
        self._aimed_debounce.calculate(False)   # reinicia el debounce
        self._active = True

    def end(self, interrupted):
        self._limelight.set_fiducial_id_filter([])
        self._locked = False
        self._aimed = False
        self._active = False

    def execute(self):
        state = self.drivetrain.get_state()
        pose = state.pose
        heading = pose.rotation()

        # 1) Heading por odometría, con compensación de movimiento
        hub = (
            train_constants.red_hub_position
            if self._is_red
            else train_constants.blue_hub_position
        )
        speeds = state.speeds
        vel_field = Translation2d(speeds.vx, speeds.vy).rotateBy(heading)
        virtual_hub = hub - vel_field * train_constants.shot_time_of_flight

        delta = virtual_hub - pose.translation()
        self._distance = delta.norm()
        odom_heading = math.atan2(delta.y, delta.x) - train_constants.shooter_mount_offset

        # 2) Corrección por visión si hay un tag válido
        valid_tag = (
            self._limelight.has_target()
            and self._limelight.get_tid() in self._own_tag_ids
            and self._limelight.get_ta() >= train_constants.vision_min_tag_area
        )
        if valid_tag:
            self._target_heading = (
                heading.radians() + math.radians(self._limelight.get_tx())
            ) - train_constants.shooter_mount_offset
            self._locked = True
        elif not self._locked:
            self._target_heading = odom_heading

        # 3) Mando al swerve: el piloto traslada, el robot gira solo
        vx = -self.pilot.getLeftY() * self.max_speed
        vy = -self.pilot.getLeftX() * self.max_speed
        self.drivetrain.set_control(
            self.request
            .with_velocity_x(vx)
            .with_velocity_y(vy)
            .with_target_direction(Rotation2d(self._target_heading))
        )

        # 4) ¿Ya está apuntando?
        err = (Rotation2d(self._target_heading) - heading).radians()
        aimed_now = (
            abs(err) < math.radians(train_constants.aim_tolerance_deg)
            and abs(speeds.omega) < train_constants.aim_max_omega_rad_s
        )
        self._aimed = self._aimed_debounce.calculate(aimed_now)

        # 5) Opcional: pasar la distancia al shooter
        if self.on_distance is not None:
            self.on_distance(self._distance)

    def isFinished(self):
        return False

"""
|
|   Autonomo
|
"""

class auto_1:
    pass