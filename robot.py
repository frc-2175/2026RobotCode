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
        #x = wpimath.applyDeadband(-self.gamepad.getRawAxis(1), 0.1)* constants.humanMaxSpeed 
       # y = wpimath.applyDeadband(-self.gamepad.getRawAxis(0), 0.1)* constants.humanMaxSpeed
      #  t = wpimath.applyDeadband(-self.gamepad.getRawAxis(4), 0.1)* constants.humanMaxTurnSpeed
        x = wpimath.applyDeadband(-self.leftJoystick.getRawAxis(1), 0.1) * constants.humanMaxSpeed
        y = wpimath.applyDeadband(-self.leftJoystick.getRawAxis(0), 0.1) * constants.humanMaxSpeed
        t = wpimath.applyDeadband(-self.rightJoystick.getRawAxis(0), 0.1) * constants.humanMaxSpeed
        self.drivetrain.drive(x, y, t)

        #Intake
        if self.gamepad.getRawButton(4):
            self.intakeandshooter.setIntakeAngle(constants.intakeOutAngle)
        elif self.gamepad.getRawButton(1):
            self.intakeandshooter.setIntakeAngle(0)

        #Shoot
        if self.gamepad.getRawButton(5):
            self.intakeandshooter.setShooterSpeed(constants.shooterSpeed)
        else:
            self.intakeandshooter.setShooterSpeed(0)

        #Roller
        if self.gamepad.getRawAxis(4):
            self.intakeandshooter.setRollerSpeed(constants.rollerSpeed)
        else:
            self.intakeandshooter.setRollerSpeed(0)

        #Indexer
        if self.gamepad.getRawAxis(4):
            self.intakeandshooter.setIndexerSpeed(constants.indexerSpeed)
        else:
            self.intakeandshooter.setIndexerSpeed(0)
