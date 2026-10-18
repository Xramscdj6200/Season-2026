from commands2 import Command


class intake_auto(Command):
    FEEDER_POWER = 0.8   # AJUSTA

    def __init__(self, intake_sub):
        super().__init__()
        self.intake = intake_sub
        self.addRequirements(self.intake)

    def execute(self):
        self.intake.set_elbow_angle(155.0)
        if self.intake.arm_is_down():
            self.intake.apply_feeder(False, self.FEEDER_POWER)
        else:
            self.intake.stop_feeder()

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.intake.stop_feeder()

class intake_out(Command):
    FEEDER_POWER = 0.8   # AJUSTA

    def __init__(self, intake_sub):
        super().__init__()
        self.intake = intake_sub
        self.addRequirements(self.intake)

    def execute(self):
        # Misma regla que el intake: el feeder solo gira con el brazo abajo
        self.intake.set_elbow_angle(155.0)
        if self.intake.arm_is_down():
            self.intake.apply_feeder(True, self.FEEDER_POWER)   # True = reversa
        else:
            self.intake.stop_feeder()

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.intake.stop_feeder()

class intake_in(Command):
    FEEDER_POWER = 0.8   # AJUSTA

    def __init__(self, intake_sub):
        super().__init__()
        self.intake = intake_sub
        self.addRequirements(self.intake)

    def execute(self):
        self.intake.set_elbow_angle(0)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.intake.stop_feeder()
