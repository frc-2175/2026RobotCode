import wpilib
from subsystems.drivetrain import Drivetrain

class MyRobot(wpilib.TimedRobot):


    def robotInit(self):
        self.drivetrain = Drivetrain()
        self.leftJoystick = wpilib.Joystick(0)
        self.rightJoystick = wpilib.Joystick(1)
        self.gamepad = wpilib.Joystick(2)

    def robotPeriodic(self):
        self.drivetrain.periodic()


    def teleopPeriodic(self):
        x = -self.gamepad.getRawAxis(1) * 4
        y = -self.gamepad.getRawAxis(0) * 4
        t = -self.gamepad.getRawAxis(4) * 2
        #x = self.leftJoystick.getRawAxis(1)
        #y = self.leftJoystick.getRawAxis(0)
        #t = self.rightJoystick.getRawAxis(0)
        self.drivetrain.drive(x, y, t)
