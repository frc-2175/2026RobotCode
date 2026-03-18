import wpilib
from subsystems.drivetrain import Drivetrain
from subsystems.intakeandshooter import IntakeAndShooter
import constants
import wpimath
import math
import choreo
from utils import ntutil
import os
from typing import List, Callable, Dict
from wpilib import Alert, SmartDashboard
from utils.swerveheading import SwerveHeadingMode
import choreo.trajectory
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds


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

        #Auto
        self.trajectoryChooser = wpilib.SendableChooser()
        self.trajectory: choreo.trajectory.SwerveTrajectory | None = None
        self.trajectoryAlerts: List[Alert] = []
        self.autoTimer = wpilib.Timer()
        self.previousAutoTime: float = 0

        autont = ntutil.getFolder("Auto")
        self.autoTimerTopic = autont.getFloatTopic("Timer")
        self.autoTrajectoryTopic = autont.getStructArrayTopic("Trajectory", Pose2d)
        self.autoChassisSpeedsTopic = autont.getStructTopic("ChassisSpeeds", ChassisSpeeds)
        self.autoPoseTopic = autont.getStructTopic("Pose", Pose2d)
        
        SmartDashboard.putData("Auto Trajectory", self.trajectoryChooser)

        self.autoEvents: Dict[str, Callable[[], None]] = {
            "EventTest": lambda: self.intakeandshooter.setShooterSpeed(constants.shooterSpeed),
            "RunShooterAndIndexer": lambda: self.intakeandshooter.setShooterAndAgitator(constants.shooterSpeed, constants.agitatorSpeed),
            "StartShooting":lambda: self.intakeandshooter.setShooterSpeed(constants.shooterSpeed),
            "LowerIntake":lambda: self.intakeandshooter.setIntakePosition(constants.intakeOutAngle),
            "RunRollerWheels":lambda: self.intakeandshooter.setRollerSpeed(constants.rollerSpeed),
            "StopRollerWheels":lambda: self.intakeandshooter.setRollerSpeed(0),
        }
        self.loadChoreoTrajectories()
        #Alerts
        self.badTrajectoryAlert = Alert("Choreo path not found", Alert.AlertType.kError)

    def robotPeriodic(self):
        self.drivetrain.periodic()
        self.intakeandshooter.periodic()
        self.updateTrajectoryTelemetry()

    def autonomousInit(self): 
        self.drivetrain.setHeadingControllerMode(SwerveHeadingMode.DISABLED)

        self.trajectory = self.trajectoryChooser.getSelected()
        if self.trajectory:
            self.autoTimer.restart()
            self.previousAutoTime = 0
            firstSample = self.trajectory.sample_at(0, self.isRedAlliance())
            if firstSample:
                self.drivetrain.resetPose(firstSample.get_pose())

    def autonomousPeriodic(self):
        currentAutoTime = self.autoTimer.get()
        self.autoTimerTopic.set(currentAutoTime)

        if self.trajectory:
            sample = self.trajectory.sample_at(self.autoTimer.get(), self.isRedAlliance())

            if sample:
                self.drivetrain.followChoreoSample(sample)
                self.autoChassisSpeedsTopic.set(sample.get_chassis_speeds())
                self.autoPoseTopic.set(sample.get_pose())

                for event in self.trajectory.events:
                    if self.previousAutoTime <= event.timestamp < currentAutoTime:
                        if event.event in self.autoEvents:
                            command = self.autoEvents[event.event]
                            command()
                        else:
                            ntutil.log(f"Autonomous event not recognized; skipping: {event.event}")
            else:
                self.drivetrain.drive(0, 0, 0)

        self.previousAutoTime = currentAutoTime

    def teleopInit(self):
        self.drivetrain.setHeadingControllerMode(SwerveHeadingMode.HUMAN_DRIVERS)
        #TODO Reset Heading Controller


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
        rightTrigger = wpimath.applyDeadband(self.gamepad.getRawAxis(3), 0.1) > 0 #Unused
        runAgitatorandIndexerOut: bool = wpimath.applyDeadband(self.gamepad.getRawAxis(2), 0.1) > 0

        runFlywheel:bool = self.gamepad.getRawButton(5)
        runAgitatorIn:bool = self.gamepad.getRawButton(6)
        rollerSpeed:float = wpimath.applyDeadband(self.gamepad.getRawAxis(5), 0.1)
        
        #Intake (LS)
        self.intakeandshooter.changeIntakePosition(intakePositionChange)

        #Flywheel (LB)
        if runFlywheel:
            self.intakeandshooter.setShooterSpeed(constants.shooterSpeed)
        else:
            self.intakeandshooter.setShooterSpeed(0)

        autoShootReady = runFlywheel and runAgitatorIn and self.intakeandshooter.currentFlywheelSpeed >= 2000
        
        #Auto Shoot(RB) and eject fuel (LT)
        if autoShootReady:
            self.intakeandshooter.setAgitatorSpeed(constants.agitatorSpeed)
            self.intakeandshooter.setIndexerSpeed(constants.indexerSpeed)
        elif runAgitatorandIndexerOut:
            self.intakeandshooter.setAgitatorSpeed(-constants.agitatorSpeed)
            self.intakeandshooter.setIndexerSpeed(-constants.indexerSpeed)
        else:
            self.intakeandshooter.setAgitatorSpeed(0)
            self.intakeandshooter.setIndexerSpeed(0)


        #Roller(RS)
        #The proper axis for the logitech controller is 5
        self.intakeandshooter.setRollerSpeed(rollerSpeed * constants.rollerSpeed)


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
        

    def loadChoreoTrajectories(self):
        choreoDir = os.path.join(wpilib.getDeployDirectory(), "choreo")
        for idx, filename in enumerate(os.listdir(choreoDir)):
            if not os.path.isfile(os.path.join(choreoDir, filename)):
                continue
            if not filename.endswith(".traj"):
                continue
            autoName = filename.removesuffix(".traj")

            try:
                trajectory = choreo.load_swerve_trajectory(autoName)
                self.trajectoryChooser.addOption(autoName, trajectory)

                for event in trajectory.events:
                    if event.event not in self.autoEvents:
                        alert = Alert(f"Invalid event in \"{autoName}\": {event.event}", Alert.AlertType.kWarning)
                        alert.set(True)
                        self.trajectoryAlerts.append(alert)
            except ValueError as err:
                ntutil.logAlert(self.badTrajectoryAlert, err)
    
    def updateTrajectoryTelemetry(self):
        # Update
        initial_pose = None
        if self.trajectory:
            if self.isRedAlliance():
                self.autoTrajectoryTopic.set([s.flipped().get_pose() for s in self.trajectory.samples])
            else:
                self.autoTrajectoryTopic.set([s.get_pose() for s in self.trajectory.samples])

            initial_pose = self.trajectory.get_initial_pose(self.isRedAlliance())
        else:
            self.autoTrajectoryTopic.set([])
