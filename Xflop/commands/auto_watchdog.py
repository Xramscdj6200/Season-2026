from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from wpilib import Timer

from generated.tuner_constants import train_constants


class resilient_path(Command):
    def __init__(self, drivetrain, path, constraints):
        super().__init__()
        self.drivetrain = drivetrain
        self.path = path
        self.constraints = constraints
        self.addRequirements(drivetrain)
        self._inner = None
        self._stall_start = None

    def initialize(self):
        self._start_leg()

    def _start_leg(self):
        self._inner = AutoBuilder.pathfindThenFollowPath(self.path, self.constraints)
        self._inner.initialize()
        self._stall_start = None

    def execute(self):
        self._inner.execute()

        speeds = self.drivetrain.get_state().speeds
        moving = (
            (speeds.vx**2 + speeds.vy**2) ** 0.5 > train_constants.auto_stall_speed
            or abs(speeds.omega) > train_constants.auto_stall_omega
        )
        now = Timer.getFPGATimestamp()

        if moving:
            self._stall_start = None
        elif self._stall_start is None:
            self._stall_start = now
        elif now - self._stall_start > train_constants.auto_stall_timeout:
            self._inner.end(True)
            self._start_leg()  # recalcula desde donde esté ahora, rodeando obstáculos

    def isFinished(self):
        return self._inner.isFinished()

    def end(self, interrupted):
        self._inner.end(interrupted)