import rev
class IntakeAndShooter:
    def __init__(self):
        self.intakeRollerMotor = rev.SparkMax(1, rev.SparkLowLevel.MotorType.kBrushless)
        self.actuaterMotor = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
