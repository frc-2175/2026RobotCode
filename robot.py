import wpilib
from subsystems.drivetrain import Drivetrain
import constants
import wpimath

class MyRobot(wpilib.TimedRobot):


    def robotInit(self):
        self.drivetrain = Drivetrain()
        self.leftJoystick = wpilib.Joystick(0)
        self.rightJoystick = wpilib.Joystick(1)
        self.gamepad = wpilib.Joystick(2)

    def robotPeriodic(self):
        self.drivetrain.periodic()


    def teleopPeriodic(self):
        x = wpimath.applyDeadband(-self.gamepad.getRawAxis(1), 0.1) * constants.humanMaxSpeed 
        y = wpimath.applyDeadband(-self.gamepad.getRawAxis(0) , 0.1)* constants.humanMaxSpeed
        t = wpimath.applyDeadband(-self.gamepad.getRawAxis(4), 0.1)* constants.humanMaxTurnSpeed
        #x = self.leftJoystick.getRawAxis(1)
        #y = self.leftJoystick.getRawAxis(0)
        #t = self.rightJoystick.getRawAxis(0)
        self.drivetrain.drive(x, y, t)
