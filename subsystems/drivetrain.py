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
        self.frontLeftSwerveModule = SwerveModule(23, 17, 0)
        self.frontRightSwerveModule = SwerveModule(22, 10, 0)
        self.backLeftSwerveModule = SwerveModule(9, 12, 0)
        self.backRightSwerveModule = SwerveModule(14, 24, 0)

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

        #.set(self.odometry.getEstimatedPosition())

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