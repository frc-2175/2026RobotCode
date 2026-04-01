import rev
import configs
import ids
import utils.ntutil as ntutil
import wpimath.units
from utils import mathutil
import constants
import wpilib
from wpilib import SmartDashboard

class IntakeAndShooter:
    def __init__(self):
        self.leftShooterMotor = rev.SparkMax(ids.leftShooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.rightShooterMotor = rev.SparkMax(ids.rightShooter, rev.SparkLowLevel.MotorType.kBrushless)
        self.agitatorMotor = rev.SparkMax(ids.agitatorMotor, rev.SparkLowLevel.MotorType.kBrushless)
        self.leftIntakeMotor = rev.SparkMax(ids.leftIntake, rev.SparkLowLevel.MotorType.kBrushless)
        self.rightIntakeMotor = rev.SparkMax(ids.rightIntake, rev.SparkLowLevel.MotorType.kBrushless)
        self.rollerMotor = rev.SparkMax(ids.rollerMotor, rev.SparkLowLevel.MotorType.kBrushless)
        self.indexerMotor = rev.SparkMax(ids.indexerMotor, rev.SparkLowLevel.MotorType.kBrushless)

        self.leftShooterMotor.configure(configs.shooterMotorFollowerConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.rightShooterMotor.configure(configs.shooterMotorConfig,rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters )
        self.agitatorMotor.configure(configs.agitatorMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.indexerMotor.configure(configs.indexerMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.leftIntakeMotor.configure(configs.intakeMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.rightIntakeMotor.configure(configs.intakeFollowerMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.rollerMotor.configure(configs.rollerMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.shooterEncoder = self.rightShooterMotor.getEncoder()
        self.intakeEncoder = self.leftIntakeMotor.getEncoder()
        self.agitatorEncoder = self.agitatorMotor.getEncoder()
        self.indexerEncoder = self.indexerMotor.getEncoder()
        self.rollerEncoder = self.rollerMotor.getEncoder()

        self.intakeController = self.leftIntakeMotor.getClosedLoopController()

        nt = ntutil.Folder("IntakeAndShooter")
        self.intakePositionTopic = nt.getFloatTopic("IntakePosition")
        self.desiredIntakePositionTopic = nt.getFloatTopic("IntakePositionDesired")
        self.shooterSpeedTopic = nt.getFloatTopic("ShooterSpeed")
        self.agitatorSpeedTopic = nt.getFloatTopic("IndexerSpeed")
        self.rollerSpeedTopic = nt.getFloatTopic("RollerSpeed")
        self.indexerSpeedTopic = nt.getFloatTopic("IndexerSpeed")
        self.mech = self.Mechanism(nt.topicName("Mechanism"), wpilib.Color.kRed)
        self.rightShooterCurrentTopic = nt.getFloatTopic("RightShooterCurrent")
        self.leftShooterCurrentTopic = nt.getFloatTopic("LeftShooterCurrent")

        self.desiredIntakePosition = 0

        self.runFlywheel = False
        self.doShoot = False
        self.indexerAndAgitatorOut = False

        

    
    def changeIntakePosition(self, intakeAngle: wpimath.units.radians):
        self.setIntakePosition(self.desiredIntakePosition + intakeAngle)

    def setIntakePosition(self, intakePosition:wpimath.units.radians):
        self.desiredIntakePosition = (mathutil.clamp(intakePosition, constants.intakeInAngle, constants.intakeOutAngle))

    def setRollerSpeed(self, rollerSpeed: float):
        self.rollerMotor.set(rollerSpeed)

    def __setShooterSpeed(self, shooterMotorSpeed: float):
        self.rightShooterMotor.set(shooterMotorSpeed)

    def __setAgitatorSpeed(self, agitatorSpeed: float):
        self.agitatorMotor.set(agitatorSpeed)

    def __setIndexerSpeed(self, indexerSpeed: float):
        self.indexerMotor.set(indexerSpeed)

    def setFlywheelRunning(self, run:bool):
        self.runFlywheel = run

    def startShooting(self, doShoot:bool):
        self.doShoot = doShoot

    def runIndexerAndAgitatorOut(self, runOut:bool):
        self.indexerAndAgitatorOut = runOut



    def periodic(self):
        self.intakeController.setSetpoint(self.desiredIntakePosition, rev.SparkLowLevel.ControlType.kPosition)

        self.shooterSpeedTopic.set(self.shooterEncoder.getVelocity())
        self.intakePositionTopic.set(self.intakeEncoder.getPosition())
        self.desiredIntakePositionTopic.set(self.desiredIntakePosition)
        self.agitatorSpeedTopic.set(self.agitatorEncoder.getVelocity())
        self.indexerSpeedTopic.set(self.indexerEncoder.getVelocity())
        self.rollerSpeedTopic.set(self.rollerEncoder.getVelocity())
        self.mech.update(wheelAngle=self.shooterEncoder.getPosition())
        self.rightShooterCurrentTopic.set(self.rightShooterMotor.getOutputCurrent())
        self.leftShooterCurrentTopic.set(self.leftShooterMotor.getOutputCurrent())

        self.currentFlywheelSpeed = self.shooterEncoder.getVelocity()

        if self.runFlywheel == True and self.currentFlywheelSpeed < constants.bangBangTargetRPM:
            self.__setShooterSpeed(constants.shooterSpeed)
        else:
            self.__setShooterSpeed(0)

        autoShootReady = self.runFlywheel and self.doShoot and self.currentFlywheelSpeed >= constants.shotRPM
        if autoShootReady:
            self.__setAgitatorSpeed(constants.agitatorSpeed)
            self.__setIndexerSpeed(constants.indexerSpeed)
        elif self.indexerAndAgitatorOut:
            self.__setAgitatorSpeed(-constants.agitatorSpeed)
            self.__setIndexerSpeed(-constants.indexerSpeed)
        else:
            self.__setAgitatorSpeed(0)
            self.__setIndexerSpeed(0)


    class Mechanism:
        def __init__(self, ntName: str, wheelColor: wpilib.Color):
            canvasWidth = 1 # m
            canvasHeight = 1 # m
            self.mech = wpilib.Mechanism2d(width=canvasWidth, height=canvasHeight)
            self.root = self.mech.getRoot("FlywheelBase",
                                          x=0.5,
                                          y=0.5)
            self.flywheel = self.root.appendLigament("Flywheel",
                                                     length=wpimath.units.inchesToMeters(2),
                                                     angle=0,
                                                     color=wpilib.Color8Bit(wheelColor))

            # Draw a "wheel" in the dumbest way possible
            i1 = self.flywheel.appendLigament("I1",
                                            length=wpimath.units.inchesToMeters(2),
                                            angle=90,
                                            color=wpilib.Color8Bit(wheelColor))
            i2 = i1.appendLigament("I2",
                                   length=wpimath.units.inchesToMeters(4),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            i3 = i2.appendLigament("I3",
                                   length=wpimath.units.inchesToMeters(4),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            i4 = i3.appendLigament("I4",
                                   length=wpimath.units.inchesToMeters(4),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            i5 = i4.appendLigament("I5",
                                   length=wpimath.units.inchesToMeters(2),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            SmartDashboard.putData(ntName, self.mech)

        def update(self, wheelAngle: wpimath.units.radians):
            self.flywheel.setAngle(wpimath.units.radiansToDegrees(-wheelAngle))
