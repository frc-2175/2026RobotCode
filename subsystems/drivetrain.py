import wpilib
import rev
from hardware.swervemodule import SwerveModule
import navx
import constants
import utils.ntutil as ntutil
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
import wpimath.units
from wpimath.geometry import Rotation2d, Translation2d, Pose2d, Pose3d, Translation3d
from wpimath.estimator import SwerveDrive4PoseEstimator
from utils.swerveheading import SwerveHeadingController, SwerveHeadingMode
import math
import wpimath.filter
from utils.slew2d import SlewRateLimiter2D
from utils.mathutil import Vector2d
import ids
import choreo.trajectory
from wpimath.controller import PIDController
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField


class Drivetrain:
    def __init__(self):
        self.frontLeftSwerveModule = SwerveModule(ids.frontLeftDrive, ids.frontLeftSteer, -3 * math.pi/2)
        self.frontRightSwerveModule = SwerveModule(ids.frontRightDrive, ids.frontRightSteer, 0)
        self.backLeftSwerveModule = SwerveModule(ids.backLeftDrive, ids.backLeftSteer, -math.pi)
        self.backRightSwerveModule = SwerveModule(ids.backRightDrive, ids.backRightSteer, -math.pi/2)
        self.kinematics = SwerveDrive4Kinematics(*constants.swerveModulePositions)
        self.gyro = navx.AHRS.create_spi()

        

        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                (
                    self.frontLeftSwerveModule.getActualPosition(),
                    self.frontRightSwerveModule.getActualPosition(),
                    self.backLeftSwerveModule.getActualPosition(),
                    self.backRightSwerveModule.getActualPosition(),
                )
            ),
            Pose2d(0, 0, self.gyro.getRotation2d())
        )

        self.desiredChassisSpeeds = ChassisSpeeds()
        self.rawAccelTest = False

        self.headingController = SwerveHeadingController(
            getHeading = self.getHeading,
            getRate = self.getHeadingRate,
            mode = SwerveHeadingMode.HUMAN_DRIVERS,
        )

        self.rotationLimiter = wpimath.filter.SlewRateLimiter(constants.rotationSlewRate)
        self.velocityLimiter = SlewRateLimiter2D(constants.maxAcceleration*1/50)

        nt = ntutil.Folder("Drivetrain")
        self.desiredChassisSpeedsTopic = nt.getStructTopic("DesiredChassisSpeeds", ChassisSpeeds)
        self.newChassisSpeedsTopic = nt.getStructTopic("NewChassisSpeeds", ChassisSpeeds)
        self.desiredStatesTopic = nt.getStructArrayTopic("DesiredSwerveStates", SwerveModuleState)
        self.actualStatesTopic = nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)
        self.gyroHeadingTopic = nt.getFloatTopic("GyroHeading")
        self.robotPoseTopic = nt.getStructTopic("RobotPose", Pose2d)
        self.visionPoseTopic = nt.getStructTopic("VisionPose", Pose3d)
        self.accelerationTopic = nt.getFloatTopic("Acceleration")
        self.accelAxisTopic = nt.getStructTopic("AccelerationAxis", Translation3d)
       
        self.choreoXController = PIDController(constants.choreoTranslationP, constants.choreoTranslationI, constants.choreoTranslationD)
        self.choreoYController = PIDController(constants.choreoTranslationP, constants.choreoTranslationI, constants.choreoTranslationD)
        self.choreoHeadingController = PIDController(constants.choreoRotationP, constants.choreoRotationI,constants.choreoRotationD)
        self.choreoHeadingController.enableContinuousInput(-math.pi, math.pi)

    def periodic(self):

        moveSpeed = math.sqrt(self.desiredChassisSpeeds.vx**2 + self.desiredChassisSpeeds.vy**2)
        newTurnSpeed = self.headingController.update(moveSpeed, self.desiredChassisSpeeds.omega)
        newVelocity = self.velocityLimiter.calculate(Vector2d(self.desiredChassisSpeeds.vx, self.desiredChassisSpeeds.vy))
        newTurnSpeed = self.rotationLimiter.calculate(newTurnSpeed)

        self.xAccel = self.gyro.getRawAccelX() * 9.80665
        self.yAccel = self.gyro.getRawAccelY() * 9.80665
        self.zAccel = self.gyro.getRawAccelZ() * 9.80665

        newChassisSpeeds = ChassisSpeeds(
            newVelocity.x, newVelocity.y, newTurnSpeed
        )

        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(newChassisSpeeds)
        if self.rawAccelTest:
            raw_accel = 1 # m/s^2
            frontLeft = SwerveModuleState(0, Rotation2d())
            frontRight = SwerveModuleState(0, Rotation2d())
            backLeft = SwerveModuleState(0, Rotation2d())
            backRight = SwerveModuleState(0, Rotation2d())
            self.frontLeftSwerveModule.setDesiredState(frontLeft, raw_accel)
            self.frontRightSwerveModule.setDesiredState(frontRight, raw_accel)
            self.backLeftSwerveModule.setDesiredState(backLeft, raw_accel)
            self.backRightSwerveModule.setDesiredState(backRight, raw_accel)
        else:
            self.frontLeftSwerveModule.setDesiredState(frontLeft)
            self.frontRightSwerveModule.setDesiredState(frontRight)
            self.backLeftSwerveModule.setDesiredState(backLeft)
            self.backRightSwerveModule.setDesiredState(backRight)

        self.desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds)
        self.newChassisSpeedsTopic.set(newChassisSpeeds)
        self.desiredStatesTopic.set([frontLeft, frontRight, backLeft, backRight])
        self.actualStatesTopic.set([
            self.frontLeftSwerveModule.getActualState(),
            self.frontRightSwerveModule.getActualState(),
            self.backLeftSwerveModule.getActualState(),
            self.backRightSwerveModule.getActualState(),
        ])
        self.gyroHeadingTopic.set(self.gyro.getRotation2d().radians())

        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeftSwerveModule.getActualPosition(),
                self.frontRightSwerveModule.getActualPosition(),
                self.backLeftSwerveModule.getActualPosition(),
                self.backRightSwerveModule.getActualPosition(),
            )
        )
        

        self.robotPoseTopic.set(self.odometry.getEstimatedPosition())
        self.accelerationTopic.set(math.sqrt(self.xAccel**2 + self.yAccel**2 ))
        self.accelAxisTopic.set(Translation3d(self.xAccel, self.yAccel, self.zAccel))


    

    def drive(
        self,
        xSpeed: wpimath.units.meters_per_second,
        ySpeed: wpimath.units.meters_per_second,
        turnSpeed: wpimath.units.radians_per_second,
        fieldRelative: bool,
    ):
        if fieldRelative:
            self.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                turnSpeed,
                self.odometry.getEstimatedPosition().rotation()
            )
        else:
            self.desiredChassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, turnSpeed)
        
        
    def getHeading(self) -> Rotation2d:
        return self.odometry.getEstimatedPosition().rotation()
    
    def getHeadingRate(self) -> wpimath.units.radians_per_second:
        return wpimath.units.degreesToRadians(self.gyro.getRate())
    
    def resetHeading(self, angle:float):
        # HACK: For some reason resetRotation(0) seems to be doubling the current
        # rotation, not actually setting it to zero. This makes literally no sense.
        # resetPose does what we want though so we are using it as a workaround.
        pose = self.odometry.getEstimatedPosition()
        self.odometry.resetPose(Pose2d(pose.x, pose.y, Rotation2d(angle)))

    def resetPose(self, pose: Pose2d):
        self.odometry.resetPose(pose)

    def setHeadingControllerMode(self, mode: SwerveHeadingMode):
        self.headingController.setMode(mode)

    def followChoreoSample(self, sample: choreo.trajectory.SwerveSample):
        pose = self.odometry.getEstimatedPosition()

        self.drive(
            sample.vx + self.choreoXController.calculate(pose.X(), sample.x),
            sample.vy + self.choreoYController.calculate(pose.Y(), sample.y),
            sample.omega + self.choreoHeadingController.calculate(pose.rotation().radians(), sample.heading),
            fieldRelative= True
        )
    
    def setHeadingControllerGoal(self, angle : Rotation2d):
        self.headingController.setGoal(angle)
