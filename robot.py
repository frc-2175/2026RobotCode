import wpilib
from subsystems.drivetrain import Drivetrain
from subsystems.intakeandshooter import IntakeAndShooter
import constants
import wpimath

class MyRobot(wpilib.TimedRobot):


    def robotInit(self):
        self.drivetrain = Drivetrain()
        self.intakeandshooter = IntakeAndShooter()
        self.leftJoystick = wpilib.Joystick(0)
        self.rightJoystick = wpilib.Joystick(1)
        self.gamepad = wpilib.Joystick(2)

    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.intakeandshooter.periodic()


    def teleopPeriodic(self):
        x = wpimath.applyDeadband(-self.gamepad.getRawAxis(1), 0.1)* constants.humanMaxSpeed 
        y = wpimath.applyDeadband(-self.gamepad.getRawAxis(0), 0.1)* constants.humanMaxSpeed
        t = wpimath.applyDeadband(-self.gamepad.getRawAxis(4), 0.1)* constants.humanMaxTurnSpeed
        #x = wpimath.applyDeadband(-self.leftJoystick.getRawAxis(1), 0.1) * constants.humanMaxSpeed
       # y = wpimath.applyDeadband(-self.leftJoystick.getRawAxis(0), 0.1) * constants.humanMaxSpeed
        #t = wpimath.applyDeadband(-self.rightJoystick.getRawAxis(0), 0.1) * constants.humanMaxTurnSpeed
        self.drivetrain.drive(x, y, t)
#TODO -Make the intake adjustable but add buttons to make them snap to defult states
#       -Add a reset heading button to the bot
        #-
        leftStick = wpimath.applyDeadband(-self.gamepad.getRawAxis(1), 0.1) * 1/8
        rightTrigger = wpimath.applyDeadband(self.gamepad.getRawAxis(3), 0.1)
        leftTrigger = wpimath.applyDeadband(self.gamepad.getRawAxis(2), 0.1)
        #Intake (LS)
        self.intakeandshooter.changeIntakePosition(leftStick)

        #Flywheel (RB)
        if self.gamepad.getRawButton(6):
            self.intakeandshooter.setShooterSpeed(constants.shooterSpeed)
        else:
            self.intakeandshooter.setShooterSpeed(0)

        #Roller(RS)
        #The proper axis for the logitech controller is 5
        if self.gamepad.getRawAxis(5) < -0.1:
            self.intakeandshooter.setRollerSpeed(constants.rollerSpeed)
        elif self.gamepad.getRawAxis(5) > 0.1:
            self.intakeandshooter.setRollerSpeed(-constants.rollerSpeed)
        else:
            self.intakeandshooter.setRollerSpeed(0)

        #Indexer(RT/LT)
        if rightTrigger > 0:
            self.intakeandshooter.setIndexerSpeed(constants.indexerSpeed)
        elif leftTrigger > 0:
            self.intakeandshooter.setIndexerSpeed(-constants.indexerSpeed)
        else:
            self.intakeandshooter.setIndexerSpeed(0)

        #Reset Rotation
        if self.gamepad.getRawButtonPressed(8):
            self.drivetrain.resetHeading(0)
