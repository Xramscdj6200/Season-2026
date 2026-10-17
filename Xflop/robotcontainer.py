import commands2
from commands2.button import CommandXboxController, Trigger
from wpilib import DriverStation, SmartDashboard
from pathplannerlib.auto import AutoBuilder, NamedCommands

from generated.tuner_constants import train_constants
from subsystems.drivetrain_sub import drivetrain_subsystem
from commands.drivetrain_com import drive, idle, brake, point, seed_command, aim
from subsystems.intake_sub import intake_subsystem
from commands.intake_com import intake_movement, intake_auto, intake_out
from subsystems.shooter_sub import shooter_subsystem 
from commands.shooter_com import shooter_movement, shoot_auto
from telemetry import Telemetry


class RobotContainer:
    def __init__(self):
        self._logger = Telemetry(
            train_constants.speed_at_12_volts * train_constants.max_speed_percentage
        )

        self._pilot = CommandXboxController(train_constants.pilot_driver_controller_port)
        self._copilot = CommandXboxController(train_constants.copilot_driver_controller_port)

        self.drivetrain = drivetrain_subsystem()
        self.intake = intake_subsystem()
        self.shooter = shooter_subsystem()

        self._aim_cmd = aim(self.drivetrain, self._pilot, self.shooter.set_distance_to_hub)
        self._aim_auto = aim(self.drivetrain, _NoInput(), self.shooter.set_distance_to_hub)

        NamedCommands.registerCommand(
            "armDown",
            self.intake.runOnce(lambda: self.intake.set_elbow_angle(self.intake.ELBOW_DOWN_DEG)),
        )
        NamedCommands.registerCommand("intakeOn", intake_auto(self.intake))
        NamedCommands.registerCommand("aimOn", self._aim_auto)
        NamedCommands.registerCommand("shootOn", shoot_auto(self.shooter, self._aim_auto.ready))

        self._auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self._auto_chooser)
        SmartDashboard.putBoolean("CopilotOn", False)

        self.configureButtonBindings()

    def _active_controller(self):
        if SmartDashboard.getBoolean("CopilotOn", False):
            return self._copilot
        return self._pilot

    def _active_y(self):
        c = self._active_controller()
        if hasattr(c, "getHID"):
            return c.getHID().getYButton()
        return c.getYButton()

    def configureButtonBindings(self):
        self.drivetrain.setDefaultCommand(drive(self.drivetrain, self._pilot, self._copilot))

        Trigger(DriverStation.isDisabled).whileTrue(
            idle(self.drivetrain).ignoringDisable(True)
        )

        """
        -- Mapeo de controles --
            
        Pilot:
            Stick Izquierdo -> Drive
            Stick Derecho -> Steer
            A -> Frenar
            B -> Apuntar llantas sin mover
            Right Bump -> Orienta el robot hacia adelante
            Left Trigger -> Intake
            Left Bump -> Sube el intake
            Y -> Outtake
            Right Trigger -> Shooter

        Copilot: (solo si CopilotOn es true)
            Left Trigger -> Intake
            Left Bump -> Sube el intake
            Y -> Outtake
            Right Trigger -> Shooter

        """

        self._pilot.a().whileTrue(brake(self.drivetrain))
        self._pilot.b().whileTrue(point(self.drivetrain, self._pilot))
        self._pilot.rightBumper().onTrue(seed_command(self.drivetrain))

        self._aim_cmd = aim(self.drivetrain, self._pilot, self.shooter.set_distance_to_hub)
        Trigger(self._active_y).whileTrue(intake_out(self.intake))

        Trigger(
            lambda: (not self.shooter.is_manual())
            and self._active_controller().getRightTriggerAxis()
            > train_constants.pilot_joystick_deadband
        ).whileTrue(self._aim_cmd)

        self.shooter.setDefaultCommand(
            shooter_movement(
                self.shooter,
                self._active_controller,
                train_constants.pilot_joystick_deadband,
                self._aim_cmd.ready,        # sin paréntesis: se evalúa cada ciclo
            )
        )

        self.intake.setDefaultCommand(
            intake_movement(
                self.intake,
                self._active_controller,
                train_constants.pilot_joystick_deadband,
            )
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self):
        return self._auto_chooser.getSelected()

class _NoInput:
    """Sticks en cero, para que el apuntado en autónomo no lea el control."""
    def getLeftX(self):
        return 0.0

    def getLeftY(self):
        return 0.0