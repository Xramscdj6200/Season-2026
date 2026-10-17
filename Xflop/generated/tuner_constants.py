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

class train_constants:

    """
    |
    |   Parametros electronicos
    |
    """

    # --- Activación del PID-VA
    steer_static_ff_sign = signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN

    # --- Ganancias del steer  ---
    steer_kp = 5   # Proporcional -Corrije el error-
    steer_ki = 0   # Integral -Corrección de error persistente-
    steer_kd = 0.5   # Derivativo -Amortiguador de corrección-
    steer_ks = 0.1   # Statica -Fuerza para ganarle a la fricción-
    steer_kv = 1.16   # Voltaje -Voltaje necesario para mantener la velocidad-
    steer_ka = 0   # Aceleración -Voltaje necesario para aumentar la velocidad

    # --- Ganancias del drive ---
    drive_kp = 1
    drive_ki = 0
    drive_kd = 0
    drive_ks = 0
    drive_kv = 0.124
    drive_ka = 0

    # --- Velocidad ---
    speed_at_12_volts: units.meters_per_second = 3.82   # Velocidad máxima esperada

    max_speed_percentage = 0.8   # Del 1 al 0, porcentaje de velocidad aplicada
    max_speed_percentage_auto = 0.1
    max_angular_rate_rotations = 0.8 # Velocidad de giro, 1 = 360° por segundo

    # --- Límites de corriente ---
    steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE   # Unidad de corriente para steer
    drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE   # Unidad de corriente para drive

    drive_request_type = swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE   # Uso de voltaje (open) o sensores (closed) para impulsar

    drive_supply_current_limit = 39.0   # Consumo de bateria a motor
    steer_stator_current_limit = 60.0   # Consumo de energia para el torque

    slip_current: units.ampere = 120.0   # Limite de amperaje al que la llanta se desliza

    """
    |
    |   Componentes
    |
    """

    # --- Tipo de motor ---
    drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

    # --- Tipo de encoder ---
    steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

    # --- CanBus ---
    canbus = CANBus("")   # CANBus(nombre del CAN, Dirección al que mandar log)

    # --- Giroscopio ---
    pigeon_id = 5
    pigeon_configs = (
        configs.Pigeon2Configuration()
        .with_mount_pose(
            configs.MountPoseConfigs()
            .with_mount_pose_yaw(-89.64210510253906)
            .with_mount_pose_pitch(2.408454418182373)
            .with_mount_pose_roll(-0.39856094121932983)
        )
    )
    """
    - Eje de Swerve -

    (+) ⭡    FL                 FR
        |    ●------------------●
    E   |    |                  |
    j   |    │                  │
    e   |    │      CENTRO      │
        |    │        ●         │
    X   |    │                  │
        |    |                  |
    (-) |    ●------------------●
        |   BL                 BR 
        |
        + ━ ━ ━ ━ ━ ━ ━ ━ ━ ━ ━ ━ ━ ⮕
               (+)   Eje Y   (-)
    """
    
    # --- Modulos ---
    _front_left_drive_motor_id = 11   # Id del motor drive
    _front_left_steer_motor_id = 12   # Id del motor steer
    _front_left_encoder_id = 13   # Id del encoder de este modulo
    _front_left_encoder_offset: units.rotation = 0.40380859375   # Offset dado por Zero Encoder en Tuner X
    _front_left_steer_motor_inverted = False   # ¿Esta invertido el steer?
    _front_left_drive_motor_inverted = False   # ¿Esta invertido el drive?
    _front_left_encoder_inverted = True   # ¿Esta invertido el encoder?

    # Distancia del centro real del robot (no pigeon) hasta el centro del modulo
    _front_left_x_pos: units.meter = 0.2921
    _front_left_y_pos: units.meter = 0.2921

    _front_right_drive_motor_id = 21
    _front_right_steer_motor_id = 22
    _front_right_encoder_id = 23
    _front_right_encoder_offset: units.rotation = 0.1103515625
    _front_right_steer_motor_inverted = False
    _front_right_drive_motor_inverted = False
    _front_right_encoder_inverted = True

    _front_right_x_pos: units.meter = 0.2921
    _front_right_y_pos: units.meter = -0.2921

    _back_left_drive_motor_id = 41
    _back_left_steer_motor_id = 42
    _back_left_encoder_id = 43
    _back_left_encoder_offset: units.rotation = -0.238525390625
    _back_left_steer_motor_inverted = False
    _back_left_drive_motor_inverted = False
    _back_left_encoder_inverted = True

    _back_left_x_pos: units.meter = -0.2921
    _back_left_y_pos: units.meter = 0.2921


    _back_right_drive_motor_id = 31
    _back_right_steer_motor_id = 32
    _back_right_encoder_id = 33
    _back_right_encoder_offset: units.rotation = -0.17236328125
    _back_right_steer_motor_inverted = False
    _back_right_drive_motor_inverted = False
    _back_right_encoder_inverted = True

    _back_right_x_pos: units.meter = -0.2921
    _back_right_y_pos: units.meter = -0.2921

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

    # --- Shooter ---

    _indexer_motor_id = 27
    _shooter_motor_id = 28

    """
    |
    |   Mecanica
    |
    """

    # --- Relaciones de engranes ---
    drive_gear_ratio = 6.821052631578947   # Vueltas del motor por vuelta de la rueda
    steer_gear_ratio = 15.42857142857143   # Vueltas del motor por vuelta del módulo (azimut)
    couple_ratio = 5.4   # Vueltas del motor de drive por vuelta del azimut (acople)

    # --- Rueda ---
    wheel_radius: units.meter = inchesToMeters(1.875)

    """
    |
    |   Driving
    |
    """

    # --- Joystick ---
    pilot_driver_controller_port = 0   # USB del driver 1
    pilot_joystick_deadband = 0.1   # Amortiguador del stick para driver 1

    copilot_driver_controller_port = 1   # USB del driver 2
    copilot_joystick_deadband = 0.1   # Amortiguador del stick para driver 2

    """
    |
    |   PathPlanner
    |
    """
    # -- PID en autonomo --
    auto_translation_kp = 10.0
    auto_rotation_kp = 7.0

    """
    |
    |   Vision (Limelight)
    |
    """

    limelight_name = "limelight"

    # -- Cuánta confianza le damos a la visión al fusionarla con la odometría (metros/radianes) --
    vision_std_dev_xy = 0.5
    vision_std_dev_theta_multi_tag = 9999999    # con 2+ tags seguimos confiando en el gyro para el heading
    vision_std_dev_theta_single_tag = 9999999   # con 1 tag, nunca corregir heading por visión

    vision_max_angular_rate = rotationsToRadians(1.0)   # No fusionar visión si el robot gira muy rápido (MegaTag2 pierde precisión). rot/s, ajustar probando

    """
    |
    |   Recalculo en autonomo
    |
    """

    auto_stall_speed = 0.05   # m/s: por debajo de esto se considera "no se mueve"
    auto_stall_omega = 0.2   # rad/s
    auto_stall_timeout = 0.6   # s parado antes de recalcular

    """
    |
    |   Campo y Hub
    |
    """
    red_hub_back_tag_ids = [3, 4]
    red_hub_aimable_tag_ids = [2, 5, 8, 9, 10, 11]

    blue_hub_back_tag_ids = [19, 20]
    blue_hub_aimable_tag_ids = [18, 21, 24, 25, 26, 27]

    field_length = 16.54   # m, de tu navgrid.json
    field_width = 8.07   # m, de tu navgrid.json
    alliance_zone_depth = inchesToMeters(134.0)   # ancho de alliance zone

    vision_min_tag_area = 0.1    # Porcentaje de apriltag que debe de ver para considerarlo apriltag

    # Centro del Hub por alianza
    blue_hub_pose_xy = (4.135, 4.038)
    red_hub_pose_xy = (12.406, 4.038)

    blue_hub_position = Translation2d(4.62, 4.03)    # AJUSTA a las coordenadas reales del hub
    red_hub_position = Translation2d(11.92, 4.03)    # AJUSTA
    shot_time_of_flight = 0.6                        # s, aproximado
    aim_tolerance_deg = 2.0

    blue_safe_aim_xy = (alliance_zone_depth, field_width / 2.0)
    red_safe_aim_xy = (field_length - alliance_zone_depth, field_width / 2.0)

    shooter_mount_offset: units.radian = math.radians(180)

    aim_tolerance_deg = 2.0
    aim_max_omega_rad_s = 0.3

    heading_kp = 1.0
    heading_ki = 0.0
    heading_kd = 0.5

    """
    |
    |   Perspectiva de alianza
    |
    """

    blue_alliance_perspective_deg = 0     # Azul ve el frente a 0°
    red_alliance_perspective_deg = 180    # Rojo ve el frente a 180°

    """
    |
    |   Constructores -- Advertencia NO TOCAR --
    |
    """

    # --- Ganancias (objetos Slot0) ---
    _steer_gains = (
        configs.Slot0Configs()
        .with_k_p(steer_kp)
        .with_k_i(steer_ki)
        .with_k_d(steer_kd)
        .with_k_s(steer_ks)
        .with_k_v(steer_kv)
        .with_k_a(steer_ka)
        .with_static_feedforward_sign(steer_static_ff_sign)
    )
    _drive_gains = (
        configs.Slot0Configs()
        .with_k_p(drive_kp)
        .with_k_i(drive_ki)
        .with_k_d(drive_kd)
        .with_k_s(drive_ks)
        .with_k_v(drive_kv)
        .with_k_a(drive_ka)
    )

    # --- Configuraciones iniciales de los dispositivos ---
    _drive_initial_configs = configs.TalonFXConfiguration().with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_supply_current_limit(drive_supply_current_limit)
            .with_supply_current_limit_enable(True)
        ).with_motor_output(
            configs.MotorOutputConfigs()
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        )
    _steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(steer_stator_current_limit)
            .with_stator_current_limit_enable(True)
        ).with_motor_output(
            configs.MotorOutputConfigs()
            .with_neutral_mode(signals.NeutralModeValue.BRAKE)
        )

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

    
    _encoder_initial_configs = configs.CANcoderConfiguration()

    # --- Constantes del drivetrain completo ---
    drivetrain_constants = (
        swerve.SwerveDrivetrainConstants()
        .with_can_bus_name(canbus.name)
        .with_pigeon2_id(pigeon_id)
        .with_pigeon2_configs(pigeon_configs)
    )

    # --- Fábrica de módulos ---
    _constants_creator = (
        swerve.SwerveModuleConstantsFactory[
            configs.TalonFXConfiguration,
            configs.TalonFXConfiguration,
            configs.CANcoderConfiguration,
        ]()
        .with_drive_motor_gear_ratio(drive_gear_ratio)
        .with_steer_motor_gear_ratio(steer_gear_ratio)
        .with_coupling_gear_ratio(couple_ratio)
        .with_wheel_radius(wheel_radius)
        .with_steer_motor_gains(_steer_gains)
        .with_drive_motor_gains(_drive_gains)
        .with_steer_motor_closed_loop_output(steer_closed_loop_output)
        .with_drive_motor_closed_loop_output(drive_closed_loop_output)
        .with_slip_current(slip_current)
        .with_speed_at12_volts(speed_at_12_volts)
        .with_drive_motor_type(drive_motor_type)
        .with_steer_motor_type(steer_motor_type)
        .with_feedback_source(steer_feedback_type)
        .with_drive_motor_initial_configs(_drive_initial_configs)
        .with_steer_motor_initial_configs(_steer_initial_configs)
        .with_encoder_initial_configs(_encoder_initial_configs)
    )

    # --- Módulos (orden de argumentos: steer, drive, encoder) ---
    front_left = _constants_creator.create_module_constants(
        _front_left_steer_motor_id,
        _front_left_drive_motor_id,
        _front_left_encoder_id,
        _front_left_encoder_offset,
        _front_left_x_pos,
        _front_left_y_pos,
        _front_left_drive_motor_inverted,
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
        _front_right_drive_motor_inverted,
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
        _back_left_drive_motor_inverted,
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
        _back_right_drive_motor_inverted,
        _back_right_steer_motor_inverted,
        _back_right_encoder_inverted,
    )

class TunerSwerveDrivetrain(swerve.SwerveDrivetrain[hardware.TalonFX, hardware.TalonFX, hardware.CANcoder]):   # Inicializa la marca y modelo del hardware (drive, steer, encoder)
    def __init__(self, drivetrain_constants, arg0=None, arg1=None, arg2=None, arg3=None):
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