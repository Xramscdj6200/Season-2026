import wpilib
import commands2
from phoenix6 import SignalLogger
import traceback

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        try:
            SignalLogger.enable_auto_logging(False)
            self.autonomousCommand = None
            self.container = RobotContainer()
        except Exception:
            print("!!! EXCEPCIÓN EN robotInit !!!")
            traceback.print_exc()
            raise

    def autonomousInit(self):
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            commands2.CommandScheduler.getInstance().schedule(self.autonomousCommand)

    def teleopInit(self):
        # Detiene el autónomo al empezar teleop
        if self.autonomousCommand:
            commands2.CommandScheduler.getInstance().cancel(self.autonomousCommand)

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)


"""

Comando para compilar el codigo al robot:
py -3 -m robotpy deploy

Comando para descargar las librerias (con internet):
py -3 -m robotpy sync

Comando para probar en un entorno simulado: syntaxis y librerias:
py -3 -m robotpy test

"""