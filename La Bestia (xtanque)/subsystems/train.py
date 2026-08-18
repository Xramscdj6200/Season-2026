import commands2
from wpimath.filter import SlewRateLimiter
from generated.constants import train_constants as consts
from wpilib import SmartDashboard

import rev


class trainSub(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.leftMotor = rev.SparkMax(consts._LmainMotor_Id, rev.SparkLowLevel.MotorType.kBrushed)
        self.rightMotor = rev.SparkMax(consts._RmainMotor_Id, rev.SparkLowLevel.MotorType.kBrushed)

        self.Lfollower = rev.SparkMax(consts._LfollowerMotor_Id, rev.SparkLowLevel.MotorType.kBrushed)
        self.Rfollower = rev.SparkMax(consts._RfollowerMotor_Id, rev.SparkLowLevel.MotorType.kBrushed)

        self._left_limitedSpeed = SlewRateLimiter(consts._acceleration)
        self._right_limitedSpeed = SlewRateLimiter(consts._acceleration)

    def forward(self, lBump, rBump):

        LeftPot = self._left_limitedSpeed.calculate(consts._desPot) + ((rBump * consts._desTurn) - (lBump * consts._desTurn))
        RightPot = self._right_limitedSpeed.calculate(consts._desPot) + ((lBump * consts._desTurn) - (rBump * consts._desTurn))

        self.leftMotor.set(LeftPot)
        self.Lfollower.set(LeftPot)
        self.rightMotor.set(-RightPot)
        self.Rfollower.set(-RightPot)

    def reverse(self, lBump, rBump):
    
            LeftPot = self._left_limitedSpeed.calculate(consts._desPot) + ((rBump * consts._desTurn) - (lBump * consts._desTurn))
            RightPot = self._right_limitedSpeed.calculate(consts._desPot) + ((lBump * consts._desTurn) - (rBump * consts._desTurn))
    
            self.leftMotor.set(-(LeftPot + consts._Loffset))
            self.Lfollower.set(-(LeftPot + consts._Loffset))
            self.rightMotor.set(RightPot)
            self.Rfollower.set(RightPot)

    def stop(self):
        left_end = 0.0
        right_end = 0.0

        self.leftMotor.set(left_end + consts._Loffset)
        self.Lfollower.set(left_end + consts._Loffset)
        self.rightMotor.set(right_end)
        self.Rfollower.set(right_end)

    def periodic(self):
        SmartDashboard.putNumber("Velocidad Motor Izq", self.leftMotor.getAppliedOutput())
        SmartDashboard.putNumber("Velocidad Motor Der", self.rightMotor.getAppliedOutput())