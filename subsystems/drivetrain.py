import wpilib
import rev
from hardware.swervemodule import SwerveModule
import navx
import constants
import ntutil
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
import wpimath.units
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator


class Drivetrain:
    def __init__(self):
        self.frontLeftSwerveModule = SwerveModule(28, 22, wpimath.units.degreesToRadians(90))
        self.frontRightSwerveModule = SwerveModule(27, 23, 0)
        self.backLeftSwerveModule = SwerveModule(25, 21, 0)
        self.backRightSwerveModule = SwerveModule(26, 24, wpimath.units.degreesToRadians(90))

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

        self.gyro = navx.AHRS.create_spi()
        self.desiredChassisSpeeds = ChassisSpeeds()

        self.kinematics = SwerveDrive4Kinematics(*constants.swerveModulePositions)

        nt = ntutil.Folder("Drivetrain")
        self.desiredChassisSpeedsTopic = nt.getStructTopic("DesiredChassisSpeeds", ChassisSpeeds)
        self.desiredStatesTopic = nt.getStructArrayTopic("DesiredSwerveStates", SwerveModuleState)
        self.actualStatesTopic = nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)
        self.gyroHeadingTopic = nt.getFloatTopic("GyroHeading")

    def periodic(self):
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(self.desiredChassisSpeeds)
        self.frontLeftSwerveModule.setDesiredState(frontLeft)
        self.frontRightSwerveModule.setDesiredState(frontRight)
        self.backLeftSwerveModule.setDesiredState(backLeft)
        self.backRightSwerveModule.setDesiredState(backRight)

        self.desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds)
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
            self.gyro.getRotation2d()
            )