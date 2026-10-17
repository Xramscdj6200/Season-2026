from commands2 import Command


class intake_auto(Command):
    FEEDER_POWER = 0.8   # AJUSTA

    def __init__(self, intake_sub):
        super().__init__()
        self.intake = intake_sub
        self.addRequirements(self.intake)

    def execute(self):
        self.intake.set_elbow_angle(self.intake.ELBOW_DOWN_DEG)
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
        self.intake.set_elbow_angle(self.intake.ELBOW_DOWN_DEG)
        if self.intake.arm_is_down():
            self.intake.apply_feeder(True, self.FEEDER_POWER)   # True = reversa
        else:
            self.intake.stop_feeder()

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.intake.stop_feeder()

class intake_movement(Command):
    def __init__(self, intake_sub, intake_pilot, pilot_deadband):
        super().__init__()
        self.intake = intake_sub
        self.pilot = intake_pilot
        self.pilot_deadband = pilot_deadband
        self.addRequirements(self.intake)

    @property
    def intake_down(self):
        return self.intake.arm_is_down()
    

    def _controller(self):
        return self.pilot() if callable(self.pilot) else self.pilot

    def _left_bumper(self, controller):
        if hasattr(controller, "getHID"):
            return controller.getHID().getLeftBumper()
        return controller.getLeftBumper()

    def in_take(self, power):
        self.intake.apply_feeder(False, power)

    def pullup_arm(self):
        self.intake.set_elbow_angle(self.intake.ELBOW_UP_DEG)

    def pushdown_arm(self):
        self.intake.set_elbow_angle(self.intake.ELBOW_DOWN_DEG)

    def shooting_sequence(self):
        pass

    def execute(self):
        controller = self._controller()
        trigger = controller.getLeftTriggerAxis()

        if self._left_bumper(controller):
            self.intake.stop_feeder()
            self.pullup_arm()

        elif trigger > self.pilot_deadband:
            self.pushdown_arm()
            if self.intake_down:
                self.in_take(trigger)
            else:
                self.intake.stop_feeder()

        else:
            self.intake.stop_feeder()

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.intake.stop_feeder()