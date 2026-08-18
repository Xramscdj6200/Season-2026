import commands2
import wpilib
from robotcontainer import RobotContainer

class Robot(commands2.TimedCommandRobot):

    def robotInit(self):
        self.container = RobotContainer()

    def robotPeriodic(self):
        pass

    def autonomousInit(self):
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        pass

    # En esta parte el teleoperado se asegura que el autonomo no siga corriendo
    def teleopInit(self):
        if hasattr(self, "autonomous_command") and self.autonomous_command:
            self.autonomous_command.cancel()

    def teleopPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)

# Abajo esta el comando para compilar el codigo al robot
#py -3 -m robotpy deploy