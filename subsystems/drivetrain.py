import wpilib
import rev
from hardware.swervemodule import SwerveModule
import navx
import constants
import utils.ntutil as ntutil
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
import wpimath.units
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from utils.swerveheading import SwerveHeadingController, SwerveHeadingMode
import math
import wpimath.filter
from utils.slew2d import SlewRateLimiter2D
from utils.mathutil import Vector2d
import ids


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

        self.headingController = SwerveHeadingController(
            getHeading = self.getHeading,
            getRate = self.getHeadingRate,
            mode = SwerveHeadingMode.HUMAN_DRIVERS,
        )

        self.roatationLimiter = wpimath.filter.SlewRateLimiter(constants.rotationSlewRate)
        #TODO Acutual Rate Limit
        self.velocityLimiter = SlewRateLimiter2D(constants.maxAcceleration*1/50)

        nt = ntutil.Folder("Drivetrain")
        self.desiredChassisSpeedsTopic = nt.getStructTopic("DesiredChassisSpeeds", ChassisSpeeds)
        self.newChassisSpeedsTopic = nt.getStructTopic("NewChassisSpeeds", ChassisSpeeds)
        self.desiredStatesTopic = nt.getStructArrayTopic("DesiredSwerveStates", SwerveModuleState)
        self.actualStatesTopic = nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)
        self.gyroHeadingTopic = nt.getFloatTopic("GyroHeading")
        self.robotPoseTopic = nt.getStructTopic("RobotPose", Pose2d)

    def periodic(self):

        moveSpeed = math.sqrt(self.desiredChassisSpeeds.vx**2 + self.desiredChassisSpeeds.vy**2)
        newTurnSpeed = self.headingController.update(moveSpeed, self.desiredChassisSpeeds.omega)
        newVelocity = self.velocityLimiter.calculate(Vector2d(self.desiredChassisSpeeds.vx, self.desiredChassisSpeeds.vy))

        newTurnSpeed = self.roatationLimiter.calculate(newTurnSpeed)

        newChassisSpeeds = ChassisSpeeds(
            newVelocity.x, newVelocity.y, newTurnSpeed
        )

        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(newChassisSpeeds)
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

    def drive(
        self,
        xSpeed: wpimath.units.meters_per_second,
        ySpeed: wpimath.units.meters_per_second,
        turnSpeed: wpimath.units.radians_per_second,
    ):
        self.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            turnSpeed,
            self.odometry.getEstimatedPosition().rotation()
        )
        
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
        # self.odometry.resetRotation(Rotation2d(angle))