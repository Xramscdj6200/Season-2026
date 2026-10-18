from commands2 import Subsystem
from wpilib import SmartDashboard
from generated.tuner_constants import TunerConstants

intake_elbow = TunerConstants.intake_elbow
intake_feeder = TunerConstants.intake_feeder


class intake_subsystem(Subsystem):
    ELBOW_UP_DEG = intake_elbow.max_deg
    ELBOW_DOWN_DEG = intake_elbow.min_deg

    def __init__(self):
        super().__init__()

    def arm_is_down(self, tolerance_deg=5.0):
        return intake_elbow.get_angle() <= self.ELBOW_DOWN_DEG + tolerance_deg

    def apply_feeder(self, direction, power):
        intake_feeder.set_percent(power * (-1 if direction else 1))

    def stop_feeder(self):
        intake_feeder.stop()

    def apply_elbow(self, direction, power):
        # direction True = subir, False = bajar
        intake_elbow.jog(direction, power)

    def set_elbow_angle(self, deg):
        intake_elbow.set_angle(deg)

    def elbow_at(self, deg):
        return intake_elbow.at_angle(deg)

    def periodic(self):
        left = intake_elbow.left.get_angle()
        right = intake_elbow.right.get_angle()
        error = left - right
        SmartDashboard.putNumber("Intake/ElbowAngle", (left + right) / 2.0)
        SmartDashboard.putNumber("Intake/ElbowLeftAngle", left)
        SmartDashboard.putNumber("Intake/ElbowRightAngle", right)
        SmartDashboard.putNumber("Intake/ElbowSetpoint", intake_elbow.get_setpoint())
        SmartDashboard.putNumber("Intake/ElbowSyncError", error)
        SmartDashboard.putBoolean("Intake/ElbowSynced", abs(error) < intake_elbow.sync_tolerance)