import wpilib
import rev
from hardware.swervemodule import SwerveModule
import navx
import constants
import ntutil
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
import wpimath.units
from wpimath.geometry import Rotation2d, Translation2d


class Drivetrain:
    def __init__(self):
        self.frontLeftSwerveModule = SwerveModule(1, 2, 0)
        self.frontRightSwerveModule = SwerveModule(3, 4, 0)
        self.backLeftSwerveModule = SwerveModule(5, 6 , 0)
        self.backRightSwerveModule = SwerveModule(7, 8, 0)
        self.gyro = navx.AHRS.create_spi()
        self.desiredChassisSpeeds = ChassisSpeeds()

        wd = constants.wheelDistanceFromCenter #Check that this value is correct
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(wd, wd),
            Translation2d(wd, -wd),
            Translation2d(-wd, wd),
            Translation2d(-wd, -wd),
        )

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