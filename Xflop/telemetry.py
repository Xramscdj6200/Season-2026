from ntcore import NetworkTableInstance
from phoenix6 import swerve, units
from wpilib import Color, Color8Bit, Mechanism2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState


class Telemetry:
    def __init__(self, max_speed: units.meters_per_second):
        self._max_speed = max_speed

        inst = NetworkTableInstance.getDefault()

        # Estado del drivetrain
        drive_state = inst.getTable("DriveState")
        self._pose = drive_state.getStructTopic("Pose", Pose2d).publish()
        self._speeds = drive_state.getStructTopic("Speeds", ChassisSpeeds).publish()
        self._states = drive_state.getStructArrayTopic("ModuleStates", SwerveModuleState).publish()
        self._targets = drive_state.getStructArrayTopic("ModuleTargets", SwerveModuleState).publish()
        self._positions = drive_state.getStructArrayTopic("ModulePositions", SwerveModulePosition).publish()
        self._timestamp = drive_state.getDoubleTopic("Timestamp").publish()
        self._odometry_frequency = drive_state.getDoubleTopic("OdometryFrequency").publish()

        # Pose para el campo (Field2d)
        pose_table = inst.getTable("Pose")
        self._field_pose = pose_table.getDoubleArrayTopic("robotPose").publish()
        self._field_type = pose_table.getStringTopic(".type").publish()

        # Un Mechanism2d por módulo: flecha de velocidad y línea de dirección
        self._module_mechanisms = []
        self._speed_ligaments = []
        self._direction_ligaments = []
        for i in range(4):
            mechanism = Mechanism2d(1, 1)
            self._speed_ligaments.append(
                mechanism.getRoot("RootSpeed", 0.5, 0.5).appendLigament("Speed", 0.5, 0)
            )
            self._direction_ligaments.append(
                mechanism.getRoot("RootDirection", 0.5, 0.5)
                .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite))
            )
            self._module_mechanisms.append(mechanism)
            SmartDashboard.putData(f"Module {i}", mechanism)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        # NetworkTables (Transmisión en vivo para Elastic/SmartDashboard)
        self._pose.set(state.pose)
        self._speeds.set(state.speeds)
        self._states.set(state.module_states)
        self._targets.set(state.module_targets)
        self._positions.set(state.module_positions)
        self._timestamp.set(state.timestamp)
        self._odometry_frequency.set(1.0 / state.odometry_period)

        # Field2d
        self._field_type.set("Field2d")
        self._field_pose.set([state.pose.x, state.pose.y, state.pose.rotation().degrees()])

        # Mechanism2d de cada módulo
        for i, module_state in enumerate(state.module_states):
            angle = module_state.angle.degrees()
            self._speed_ligaments[i].setAngle(angle)
            self._direction_ligaments[i].setAngle(angle)
            self._speed_ligaments[i].setLength(module_state.speed / (2 * self._max_speed))