from commands2 import Command
from wpilib import Timer

from wpilib import Timer

class shoot_auto(Command):
    """Dispara en autónomo: sube el shooter y alimenta cuando está listo."""

    def __init__(self, shooter_sub, ready_supplier=None):
        super().__init__()
        self.shooter = shooter_sub
        self.ready_supplier = ready_supplier
        self._timer = Timer()
        self.addRequirements(self.shooter)

    def _ready_to_feed(self):
        spun_up = self._timer.hasElapsed(shooter_movement.SPINUP_S)
        if self.shooter.is_manual():
            return spun_up
        aimed = self.ready_supplier() if self.ready_supplier is not None else True
        return spun_up and aimed

    def initialize(self):
        self._timer.restart()

    def execute(self):
        self.shooter.shooter_movement()
        if self._ready_to_feed():
            self.shooter.indexer_movement(shooter_movement.INDEXER_PERCENT)
        else:
            self.shooter.stop_indexer()

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.shooter.stop_shooter()
        self.shooter.stop_indexer()

class shooter_movement(Command):
    SPINUP_S = 0.8            # AJUSTA: tiempo para que el shooter llegue a velocidad
    INDEXER_PERCENT = 0.6     # AJUSTA

    def __init__(self, shooter_sub, shooter_pilot, pilot_deadband, ready_supplier=None):
        super().__init__()
        self.shooter = shooter_sub
        self.pilot = shooter_pilot
        self.pilot_deadband = pilot_deadband
        self.ready_supplier = ready_supplier       # función que devuelve bool
        self._spinup_timer = Timer()
        self._shooting = False
        self.addRequirements(self.shooter)

    def _ready_to_feed(self):
        spun_up = self._spinup_timer.hasElapsed(self.SPINUP_S)
        aimed = self.ready_supplier() if self.ready_supplier is not None else True
        return spun_up and aimed

    def _controller(self):
        return self.pilot() if callable(self.pilot) else self.pilot

    def _ready_to_feed(self):
        spun_up = self._spinup_timer.hasElapsed(self.SPINUP_S)
        if self.shooter.is_manual():
            return spun_up
        aimed = self.ready_supplier() if self.ready_supplier is not None else True
        return spun_up and aimed

    def _stop(self):
        self._shooting = False
        self._spinup_timer.stop()
        self._spinup_timer.reset()
        self.shooter.stop_shooter()
        self.shooter.stop_indexer()

    def initialize(self):
        self._stop()

    def execute(self):
        pressed = self._controller().getRightTriggerAxis() > self.pilot_deadband

        if pressed:
            if not self._shooting:
                self._shooting = True
                self._spinup_timer.restart()

            self.shooter.shooter_movement()
            if self._ready_to_feed():
                self.shooter.indexer_movement(self.INDEXER_PERCENT)
            else:
                self.shooter.stop_indexer()
        elif self._shooting:
            self._stop()

    def isFinished(self):
        return False

    def end(self, interrupted):
        self._stop()