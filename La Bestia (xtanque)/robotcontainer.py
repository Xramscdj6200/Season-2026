import wpilib
import commands2
from commands2.button import CommandXboxController
from subsystems.train import trainSub

class ApplyCommand(commands2.Command):

    def __init__(self, train, controller, action):
        super().__init__()

        self.train = train
        self._joystick = controller
        self._action = action
        self.isActive = False

        self.addRequirements(train)

    def setAction(self, action):
        if self.isActive == False:
            self.isActive = True
            self._action = action
        else:
            self.isActive = False
            self._action = ""


    def execute(self):

        if not self._joystick: return

        self.train.periodic()
        
        if self._action == "a":
            self.train.forward(self._joystick.getHID().getLeftTriggerAxis(), self._joystick.getHID().getRightTriggerAxis())
        elif self._action == "b":
            self.train.reverse(self._joystick.getHID().getLeftTriggerAxis(), self._joystick.getHID().getRightTriggerAxis())
        else:
            self.train.stop()

    def end(self, interrupted):
        self.train.stop()

    def isFinished(self):
        return False

class RobotContainer:
    def __init__(self):
        self.train = trainSub()
        
        self._joystick = CommandXboxController(0)
        self.apply = ApplyCommand(self.train, self._joystick, "")

        self.configure_button_bindings()

    def configure_button_bindings(self):
        self._joystick.a().onTrue(
            commands2.cmd.runOnce(lambda: self.apply.setAction("a"))
        )

        self._joystick.b().onTrue(
            commands2.cmd.runOnce(lambda: self.apply.setAction("b"))
        )

        self.train.setDefaultCommand(self.apply)

        return None

    def get_autonomous_command(self):
        return None