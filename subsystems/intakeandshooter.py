import rev
import configs
import ids
class IntakeAndShooter:
    def __init__(self):
        self.leftShooterMotor = rev.SparkMax(ids.leftShooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.rightShooterMotor = rev.SparkMax(ids.rightShooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.indexerMotor = rev.SparkMax(ids.indexerMotor, rev.SparkLowLevel.MotorType.kBrushless)
        self.leftIntakeMotor = rev.SparkMax(ids.leftIntake, rev.SparkLowLevel.MotorType.kBrushless)
        self.rightIntakeMotor = rev.SparkMax(ids.rightIntake, rev.SparkLowLevel.MotorType.kBrushless)
        self.rollerMotor = rev.SparkMax(ids.rollerMotor, rev.SparkLowLevel.MotorType.kBrushless)

        self.leftShooterMotor.configure(configs.shooterMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.rightShooterMotor.configure(configs.shooterMotorFollowerConfig,rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters )
        self.indexerMotor.configure(configs.indexerMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.leftIntakeMotor.configure(configs.intakeMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.rightIntakeMotor.configure(configs.intakeFollowerMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.rollerMotor.configure(configs.rollerMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

    
    def setIntakeSpeed(self, intakeSpeed: float):
        self.leftIntakeMotor.set(intakeSpeed)

    def setRollerSpeed(self, rollerSpeed: float):
        self.rollerMotor.set(rollerSpeed)

    def setShooterSpeed(self, shooterMotorSpeed: float):
        self.leftShooterMotor.set(shooterMotorSpeed)

    def setIndexerSpeed(self, indexerSpeed: float):
        self.indexerMotor.set(indexerSpeed)

    def periodic(self):
        pass
