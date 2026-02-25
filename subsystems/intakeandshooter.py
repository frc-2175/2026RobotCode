import rev
import configs
import ids
import utils.ntutil as ntutil
import wpimath.units
from utils import mathutil
import constants
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

        self.shooterEncoder = self.leftShooterMotor.getEncoder()
        self.intakeEncoder = self.leftIntakeMotor.getEncoder()
        self.indexerEncoder = self.indexerMotor.getEncoder()
        self.rollerEncoder = self.rollerMotor.getEncoder()

        self.intakeController = self.leftIntakeMotor.getClosedLoopController()

        nt = ntutil.Folder("IntakeAndShooter")
        self.intakePositionTopic = nt.getFloatTopic("IntakePosition",)
        self.shooterSpeedTopic = nt.getFloatTopic("ShooterSpeed")
        self.indexerSpeedTopic = nt.getFloatTopic("IndexerSpeed")
        self.rollerSpeedTopic = nt.getFloatTopic("RollerSpeed")

        self.desiredIntakePosition = 0

        

    
    def changeIntakePosition(self, intakeAngle: wpimath.units.radians):
        self.desiredIntakePosition = self.desiredIntakePosition + intakeAngle
        self.desiredIntakePosition = (mathutil.clamp(self.desiredIntakePosition, 0, constants.intakeOutAngle))

    def setRollerSpeed(self, rollerSpeed: float):
        self.rollerMotor.set(rollerSpeed)

    def setShooterSpeed(self, shooterMotorSpeed: float):
        self.leftShooterMotor.set(shooterMotorSpeed)

    def setIndexerSpeed(self, indexerSpeed: float):
        self.indexerMotor.set(indexerSpeed)

    def setShooterAndIndexer(self, shooterSpeed: float, indexerSpeed: float):
        self.leftShooterMotor.set(shooterSpeed)
        self.indexerMotor.set(indexerSpeed)

    def periodic(self):
        self.intakeController.setSetpoint(self.desiredIntakePosition, rev.SparkLowLevel.ControlType.kPosition)

        self.shooterSpeedTopic.set(self.shooterEncoder.getVelocity())
        self.intakePositionTopic.set(self.intakeEncoder.getPosition())
        self.indexerSpeedTopic.set(self.indexerEncoder.getVelocity())
        self.rollerSpeedTopic.set(self.rollerEncoder.getVelocity())
