from typing import TYPE_CHECKING, overload

if TYPE_CHECKING:
    from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain


import math
import wpilib
from phoenix6 import CANBus, configs, hardware, signals, swerve, units, controls
from wpilib import SmartDashboard, DriverStation
from wpimath.units import inchesToMeters, rotationsToRadians
from wpimath.geometry import Translation2d
from wpiutil import Sendable, SendableBuilder

def _clamp(x, lo, hi):
    return max(lo, min(hi, x))

def _inverted(flag):
    return (
        signals.InvertedValue.CLOCKWISE_POSITIVE
        if flag
        else signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )

def _apply_config(configurator, cfg, label, retries = 5) :
    """Aplica la config reintentando; avisa en la DS si falla."""
    status = None
    for _ in range(retries):
        status = configurator.apply(cfg)
        if status.is_ok():
            return True
    _report_error(f"[Intake] No se pudo configurar {label}: {status.name}")
    return False

class ElbowMotor:
    """Un solo motor del codo. Trabaja en grados del mecanismo (positivo = arriba)."""

    def __init__(self, motor_id, inverted, can_bus, gear_ratio,
                 min_deg, max_deg, start_deg, kp, ki, kd, stator_limit_amps=30.0):
        self._motor = hardware.TalonFX(motor_id, can_bus)

        cfg = configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        cfg.motor_output.inverted = _inverted(inverted)
        cfg.current_limits.stator_current_limit = stator_limit_amps
        cfg.current_limits.stator_current_limit_enable = True

        # La posición del TalonFX queda en rotaciones del MECANISMO (no del motor)
        cfg.feedback.sensor_to_mechanism_ratio = gear_ratio

        # Freno real: soft limits en grados -> rotaciones del mecanismo
        cfg.software_limit_switch.forward_soft_limit_enable = True
        cfg.software_limit_switch.forward_soft_limit_threshold = max_deg / 360.0
        cfg.software_limit_switch.reverse_soft_limit_enable = True
        cfg.software_limit_switch.reverse_soft_limit_threshold = min_deg / 360.0

        cfg.slot0.k_p, cfg.slot0.k_i, cfg.slot0.k_d = kp, ki, kd
        _apply_config(self._motor.configurator, cfg, f"codo id {motor_id}")

        self._motor.set_position(start_deg / 360.0)  # referencia inicial conocida
        self._request = controls.PositionVoltage(0)

    def set_angle(self, deg):
        self._motor.set_control(self._request.with_position(deg / 360.0))

    def get_angle(self):
        return self._motor.get_position().value_as_double * 360.0

    def apply_pid(self, kp, ki, kd):
        slot0 = configs.Slot0Configs()
        slot0.k_p, slot0.k_i, slot0.k_d = kp, ki, kd
        self._motor.configurator.apply(slot0)

class IntakeElbow(Sendable):
    """Par de motores del codo, controlados de forma independiente al mismo ángulo."""

    def __init__(self, name, left_id, left_inverted, right_id, right_inverted,
                 kp, ki, kd, gear_ratio, min_deg, max_deg, start_deg,
                 can_bus="", max_rate_deg_s=60.0, slow_zone_deg=10.0,
                 max_lead_deg=10.0, sync_tolerance_deg=5.0):
        super().__init__()
        self.min_deg, self.max_deg = min_deg, max_deg
        self.max_rate = max_rate_deg_s
        self.slow_zone = slow_zone_deg
        self.max_lead = max_lead_deg
        self.sync_tolerance = sync_tolerance_deg
        self._pid = [kp, ki, kd]
        self._setpoint = start_deg

        self.left = ElbowMotor(left_id, left_inverted, can_bus, gear_ratio,
                               min_deg, max_deg, start_deg, kp, ki, kd)
        self.right = ElbowMotor(right_id, right_inverted, can_bus, gear_ratio,
                                min_deg, max_deg, start_deg, kp, ki, kd)

        SmartDashboard.putData(name, self)

    # ---------- Control ----------
    def set_angle(self, deg):
        """Manda ambos motores al mismo ángulo, siempre dentro de los límites."""
        self._setpoint = _clamp(deg, self.min_deg, self.max_deg)
        self.left.set_angle(self._setpoint)
        self.right.set_angle(self._setpoint)

    def jog(self, up, power, dt=0.02):
        """Mueve el setpoint a una velocidad proporcional a power (0..1).
        Con power = 0 el codo se queda sosteniendo el último setpoint."""
        power = _clamp(abs(power), 0.0, 1.0)
        angle = self.get_angle()

        # Freno imaginario: baja la velocidad al acercarse al límite
        dist = (self.max_deg - angle) if up else (angle - self.min_deg)
        scale = _clamp(dist / self.slow_zone, 0.15, 1.0) if dist > 0 else 0.0

        step = power * scale * self.max_rate * dt * (1 if up else -1)
        new_sp = self._setpoint + step
        # Evita que el setpoint se aleje demasiado del ángulo real (windup)
        new_sp = _clamp(new_sp, angle - self.max_lead, angle + self.max_lead)
        self.set_angle(new_sp)

    # ---------- Lectura ----------
    def get_angle(self):
        return (self.left.get_angle() + self.right.get_angle()) / 2.0

    def get_setpoint(self):
        return self._setpoint

    def sync_error(self):
        return self.left.get_angle() - self.right.get_angle()

    def is_synced(self):
        return abs(self.sync_error()) < self.sync_tolerance

    def at_angle(self, target, tolerance_deg=2.0):
        return abs(self.get_angle() - target) < tolerance_deg

    # ---------- PID en vivo (Elastic) ----------
    def _set_gain(self, i, value):
        if self._pid[i] != value:
            self._pid[i] = value
            self.left.apply_pid(*self._pid)
            self.right.apply_pid(*self._pid)

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("PIDController")
        builder.addDoubleProperty("p", lambda: self._pid[0], lambda v: self._set_gain(0, v))
        builder.addDoubleProperty("i", lambda: self._pid[1], lambda v: self._set_gain(1, v))
        builder.addDoubleProperty("d", lambda: self._pid[2], lambda v: self._set_gain(2, v))
        builder.addDoubleProperty("setpoint", self.get_setpoint, self.set_angle)

def _report_error(message):
    try:
        wpilib.reportError(message, False)
    except Exception:
        print(message)

class IntakeFeederMotors:
    """Par de motores del feeder: sin PID, solo voltaje / porcentaje."""

    def __init__(self, left_id, left_inverted, right_id, right_inverted,
                 can_bus="", stator_limit_amps=40.0):
        self._motors = [
            hardware.TalonFX(left_id, can_bus),
            hardware.TalonFX(right_id, can_bus),
        ]
        for motor, inverted in zip(self._motors, (left_inverted, right_inverted)):
            cfg = configs.TalonFXConfiguration()
            cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST
            cfg.motor_output.inverted = (
                signals.InvertedValue.CLOCKWISE_POSITIVE
                if inverted
                else signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
            cfg.current_limits.stator_current_limit = stator_limit_amps
            cfg.current_limits.stator_current_limit_enable = True
            _apply_config(motor.configurator, cfg, f"feeder id {motor.device_id}")

        self._duty_request = controls.DutyCycleOut(0)
        self._voltage_request = controls.VoltageOut(0)

    def set_percent(self, percent: float):
        req = self._duty_request.with_output(percent)
        for m in self._motors:
            m.set_control(req)

    def set_voltage(self, volts: float):
        req = self._voltage_request.with_output(volts)
        for m in self._motors:
            m.set_control(req)

    def stop(self):
        self.set_percent(0.0)

class TunerConstants:
    """
    Generated by the 2026 Tuner X Swerve Project Generator
    https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    """

    # Both sets of gains need to be tuned to your individual robot

    # The steer motor uses any SwerveModule.SteerRequestType control request with the
    # output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_p(10)
        .with_k_i(0)
        .with_k_d(0.5)
        .with_k_s(0.1)
        .with_k_v(1.16)
        .with_k_a(0)
        .with_static_feedforward_sign(
            signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
        )
    )
    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(0.1)
        .with_k_i(0)
        .with_k_d(0)
        .with_k_s(0)
        .with_k_v(0.124)
    )

    # The closed-loop output type to use for the steer motors;
    # This affects the PID/FF gains for the steer motors
    _steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
    # The closed-loop output type to use for the drive motors;
    # This affects the PID/FF gains for the drive motors
    _drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

    # The type of motor used for the drive motor
    _drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    # The type of motor used for the steer motor
    _steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    # The remote sensor feedback type to use for the steer motors;
    # When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    _steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

    # The stator current at which the wheels start to slip;
    # This needs to be tuned to your individual robot
    _slip_current: units.ampere = 120.0

    # Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    # Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
    _drive_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        # Default supply current limit is 70 A, but it can be lowered to avoid brownouts.
        # Supply current limits can be larger than the breaker current rating.
        .with_supply_current_limit(39.0)
        .with_supply_current_limit_enable(True)
    )
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        # Swerve azimuth does not require much torque output, so we can set a relatively low
        # stator current limit to help avoid brownouts without impacting performance.
        .with_stator_current_limit(60.0)
        .with_stator_current_limit_enable(True)
    )
    _encoder_initial_configs = configs.CANcoderConfiguration()
    # Configs for the Pigeon 2; leave this None to skip applying Pigeon 2 configs
    _pigeon_configs: configs.Pigeon2Configuration | None = None

    # CAN bus that the devices are located on;
    # All swerve devices must share the same CAN bus
    canbus = CANBus("", "./logs/example.hoot")

    # Measured robot speed (m/s) at 12 V applied output;
    # This is NOT the desired max robot speed - see _max_speed in RobotContainer instead;
    # This needs to be tuned to your individual robot
    speed_at_12_volts: units.meters_per_second = 4.13

    # Every 1 rotation of the azimuth results in _couple_ratio drive motor turns;
    # This may need to be tuned to your individual robot
    _couple_ratio = 5.4

    _drive_gear_ratio = 6.821052631578947
    _steer_gear_ratio = 12.1
    _wheel_radius: units.meter = inchesToMeters(1.825)

    _invert_left_side = False
    _invert_right_side = True

    _pigeon_id = 5

    # These are only used for simulation
    _steer_inertia: units.kilogram_square_meter = 0.01
    _drive_inertia: units.kilogram_square_meter = 0.035
    # Simulated voltage necessary to overcome friction
    _steer_friction_voltage: units.volt = 0.2
    _drive_friction_voltage: units.volt = 0.2

    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(_pigeon_id)
        .with_pigeon2_configs(_pigeon_configs)
    )

    _constants_creator = (
        swerve.SwerveModuleConstantsFactory[
            configs.TalonFXConfiguration,
            configs.TalonFXConfiguration,
            configs.CANcoderConfiguration,
        ]()
        .with_drive_motor_gear_ratio(_drive_gear_ratio)
        .with_steer_motor_gear_ratio(_steer_gear_ratio)
        .with_coupling_gear_ratio(_couple_ratio)
        .with_wheel_radius(_wheel_radius)
        .with_steer_motor_gains(_steer_gains)
        .with_drive_motor_gains(_drive_gains)
        .with_steer_motor_closed_loop_output(_steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(_drive_closed_loop_output)
        .with_slip_current(_slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(_drive_motor_type)
        .with_steer_motor_type(_steer_motor_type)
        .with_feedback_source(_steer_feedback_type)
        .with_drive_motor_initial_configs(_drive_initial_configs)
        .with_steer_motor_initial_configs(_steer_initial_configs)
        .with_encoder_initial_configs(_encoder_initial_configs)
        .with_steer_inertia(_steer_inertia)
        .with_drive_inertia(_drive_inertia)
        .with_steer_friction_voltage(_steer_friction_voltage)
        .with_drive_friction_voltage(_drive_friction_voltage)
    )


    # Front Left
    _front_left_drive_motor_id = 11
    _front_left_steer_motor_id = 12
    _front_left_encoder_id = 13
    _front_left_encoder_offset: units.rotation = -0.09326171875
    _front_left_steer_motor_inverted = False
    _front_left_encoder_inverted = True

    _front_left_x_pos: units.meter = inchesToMeters(11.25)
    _front_left_y_pos: units.meter = inchesToMeters(11.25)

    # Front Right
    _front_right_drive_motor_id = 21
    _front_right_steer_motor_id = 22
    _front_right_encoder_id = 23
    _front_right_encoder_offset: units.rotation = -0.390869140625
    _front_right_steer_motor_inverted = False
    _front_right_encoder_inverted = True

    _front_right_x_pos: units.meter = inchesToMeters(11.25)
    _front_right_y_pos: units.meter = inchesToMeters(-11.25)

    # Back Left
    _back_left_drive_motor_id = 41
    _back_left_steer_motor_id = 42
    _back_left_encoder_id = 43
    _back_left_encoder_offset: units.rotation = 0.273681640625
    _back_left_steer_motor_inverted = False
    _back_left_encoder_inverted = True

    _back_left_x_pos: units.meter = inchesToMeters(-11.25)
    _back_left_y_pos: units.meter = inchesToMeters(11.25)

    # Back Right
    _back_right_drive_motor_id = 31
    _back_right_steer_motor_id = 32
    _back_right_encoder_id = 33
    _back_right_encoder_offset: units.rotation = 0.338623046875
    _back_right_steer_motor_inverted = False
    _back_right_encoder_inverted = True

    _back_right_x_pos: units.meter = inchesToMeters(-11.25)
    _back_right_y_pos: units.meter = inchesToMeters(-11.25)


    front_left = _constants_creator.create_module_constants(
        _front_left_steer_motor_id,
        _front_left_drive_motor_id,
        _front_left_encoder_id,
        _front_left_encoder_offset,
        _front_left_x_pos,
        _front_left_y_pos,
        _invert_left_side,
        _front_left_steer_motor_inverted,
        _front_left_encoder_inverted,
    )
    front_right = _constants_creator.create_module_constants(
        _front_right_steer_motor_id,
        _front_right_drive_motor_id,
        _front_right_encoder_id,
        _front_right_encoder_offset,
        _front_right_x_pos,
        _front_right_y_pos,
        _invert_right_side,
        _front_right_steer_motor_inverted,
        _front_right_encoder_inverted,
    )
    back_left = _constants_creator.create_module_constants(
        _back_left_steer_motor_id,
        _back_left_drive_motor_id,
        _back_left_encoder_id,
        _back_left_encoder_offset,
        _back_left_x_pos,
        _back_left_y_pos,
        _invert_left_side,
        _back_left_steer_motor_inverted,
        _back_left_encoder_inverted,
    )
    back_right = _constants_creator.create_module_constants(
        _back_right_steer_motor_id,
        _back_right_drive_motor_id,
        _back_right_encoder_id,
        _back_right_encoder_offset,
        _back_right_x_pos,
        _back_right_y_pos,
        _invert_right_side,
        _back_right_steer_motor_inverted,
        _back_right_encoder_inverted,
    )

    # --- Subsistemas ---

    _intake_elbow_kp = 0.25
    _intake_elbow_ki = 0.0
    _intake_elbow_kd = 0.1

    _left_intake_elbow_motor_id = 17
    _left_intake_elbow_motor_inverted = False

    _right_intake_elbow_motor_id = 18
    _right_intake_elbow_motor_inverted = True

    _left_intake_feeder_motor_id = 19
    _left_intake_feeder_motor_inverted = False

    _right_intake_feeder_motor_id = 20
    _right_intake_feeder_motor_inverted = True

    _intake_elbow_min_degrees = 0.0
    _intake_elbow_max_degrees = 110.0

    intake_elbow = IntakeElbow(
        "Intake/ElbowPID",
        _left_intake_elbow_motor_id, _left_intake_elbow_motor_inverted, _right_intake_elbow_motor_id, _right_intake_elbow_motor_inverted,
        _intake_elbow_kp, _intake_elbow_ki, _intake_elbow_kd,
        gear_ratio = 50.0,   # AJUSTA: vueltas del motor por vuelta del codo
        min_deg = _intake_elbow_min_degrees,
        max_deg = _intake_elbow_max_degrees,
        start_deg = _intake_elbow_max_degrees,   # arranca recogido
    )

    intake_feeder = IntakeFeederMotors(_left_intake_feeder_motor_id, _left_intake_feeder_motor_inverted, _right_intake_feeder_motor_id, _right_intake_feeder_motor_inverted)


    # --- Shooter ---

    _indexer_motor_id = 27
    _shooter_motor_id = 28

    @classmethod
    def create_drivetrain(cls) -> "CommandSwerveDrivetrain":
        """
        Creates a CommandSwerveDrivetrain instance.
        This should only be called once in your robot program.
        """
        from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

        return CommandSwerveDrivetrain(
            cls.drivetrain_constants,
            [
                cls.front_left,
                cls.front_right,
                cls.back_left,
                cls.back_right,
            ],
        )


class TunerSwerveDrivetrain(
    swerve.SwerveDrivetrain[hardware.TalonFX, hardware.TalonFX, hardware.CANcoder]
):
    """Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types."""

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0: None,
        arg1: None,
        arg2: None,
        arg3: None,
        /,
    ) -> None: ...

    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        swerve.SwerveDrivetrain.__init__(
            self,
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            drivetrain_constants,
            arg0,
            arg1,
            arg2,
            arg3,
        )