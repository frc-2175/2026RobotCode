import wpilib
from subsystems.drivetrain import Drivetrain
from subsystems.intakeandshooter import IntakeAndShooter
import constants
import wpimath
import math

class MyRobot(wpilib.TimedRobot):


    def robotInit(self):
        self.drivetrain = Drivetrain()
        self.intakeandshooter = IntakeAndShooter()
        self.leftJoystick = wpilib.Joystick(0)
        self.rightJoystick = wpilib.Joystick(1)
        self.gamepad = wpilib.Joystick(2)

        # NOTE(ben): In real competition scenarios this will not really be necessary,
        # because autonomous will necessarily reset the estimated pose to whatever is
        # at the start of the auto routine. Plus, if we have vision, that will be
        # continuously updating our estimated pose anyway. But for now this allows us
        # to have a sane forward direction if we go straight into teleop.
        self.drivetrain.resetHeading(self.driverForwardAngle())

    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.intakeandshooter.periodic()


    def teleopPeriodic(self):
        #x = wpimath.applyDeadband(-self.gamepad.getRawAxis(1), 0.1)* constants.humanMaxSpeed 
        #y = wpimath.applyDeadband(-self.gamepad.getRawAxis(0), 0.1)* constants.humanMaxSpeed
        #t = wpimath.applyDeadband(-self.gamepad.getRawAxis(4), 0.1)* constants.humanMaxTurnSpeed
        x = wpimath.applyDeadband(-self.leftJoystick.getRawAxis(1), 0.1) * constants.humanMaxSpeed
        y = wpimath.applyDeadband(-self.leftJoystick.getRawAxis(0), 0.1) * constants.humanMaxSpeed
        t = wpimath.applyDeadband(-self.rightJoystick.getRawAxis(0), 0.1) * constants.humanMaxTurnSpeed

        if self.isRedAlliance():
            x = -x
            y = -y

        self.drivetrain.drive(x, y, t)

        intakePositionChange: float = wpimath.applyDeadband(self.gamepad.getRawAxis(1), 0.1) * 1/8
        rightTrigger = wpimath.applyDeadband(self.gamepad.getRawAxis(3), 0.1)
        runIndexerOut: bool = wpimath.applyDeadband(self.gamepad.getRawAxis(2), 0.1) > 0

        runFlywheel:bool = self.gamepad.getRawButton(5)
        runIndexerIn:bool = self.gamepad.getRawButton(6)
        rollerSpeed:float = wpimath.applyDeadband(self.gamepad.getRawAxis(5), 0.1)
        
        #Intake (LS)
        self.intakeandshooter.changeIntakePosition(intakePositionChange)

        #Flywheel (RB)
        if runFlywheel:
            self.intakeandshooter.setShooterSpeed(constants.shooterSpeed)
        else:
            self.intakeandshooter.setShooterSpeed(0)

        #Roller(RS)
        #The proper axis for the logitech controller is 5
        self.intakeandshooter.setRollerSpeed(rollerSpeed * constants.rollerSpeed)

        #Indexer(RT/LT)
        if runIndexerIn and runFlywheel:
            self.intakeandshooter.setIndexerSpeed(constants.indexerSpeed)
        elif runIndexerOut:
            self.intakeandshooter.setIndexerSpeed(-constants.indexerSpeed)
        else:
            self.intakeandshooter.setIndexerSpeed(0)

        #Reset Rotation
        if self.leftJoystick.getRawButtonPressed(8):
            self.drivetrain.resetHeading(self.driverForwardAngle())

    def isRedAlliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    
    def driverForwardAngle(self) -> float:
        if self.isRedAlliance():
            return math.pi
        else:
            return 0
