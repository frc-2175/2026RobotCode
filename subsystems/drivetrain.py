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

        self.autonomousMode = False
        self.autoVelocityChassisSpeeds = ChassisSpeeds()
        self.autoAccelChassisSpeeds = ChassisSpeeds() # NOTE(ben): We are "lying" to kinematics about the units with this, but it's fine.

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
        self.autoVelocityChassisSpeedsTopic = nt.getStructTopic("AutoVelocityChassisSpeeds", ChassisSpeeds)
        self.autoAccelerationChassisSpeedsTopic = nt.getStructTopic("AutoAccelerationChassisSpeeds", ChassisSpeeds)
        self.frontLeftFFVelTopic = nt.getFloatTopic("FrontLeftFFVel")
        self.frontLeftFFAccTopic = nt.getFloatTopic("FrontLeftFFAcc")
        self.frontLeftFFTopic = nt.getFloatTopic("FrontLeftFF")

        self.choreoXController = PIDController(constants.choreoTranslationP, constants.choreoTranslationI, constants.choreoTranslationD)
        self.choreoYController = PIDController(constants.choreoTranslationP, constants.choreoTranslationI, constants.choreoTranslationD)
        self.choreoHeadingController = PIDController(constants.choreoRotationP, constants.choreoRotationI,constants.choreoRotationD)
        self.choreoHeadingController.enableContinuousInput(-math.pi, math.pi)

    def periodic(self):
        if self.autonomousMode:
            frontLeftVel, frontRightVel, backLeftVel, backRightVel = self.kinematics.toSwerveModuleStates(self.autoVelocityChassisSpeeds)
            frontLeftAcc, frontRightAcc, backLeftAcc, backRightAcc = self.kinematics.toSwerveModuleStates(self.autoAccelChassisSpeeds)

            frontLeftFFVel = frontLeftVel.speed / constants.physicalMaxSpeed * 12
            frontLeftFFAcc = self.frontLeftSwerveModule.accelerationToMotorVoltage(frontLeftAcc.speed) # NOTE(ben): "Speed" is in m/s^2 because it is actually acceleration :)
            frontLeftFFVoltage = frontLeftFFVel + frontLeftFFAcc

            frontRightFFVel = frontRightVel.speed / constants.physicalMaxSpeed * 12
            frontRightFFAcc = self.frontRightSwerveModule.accelerationToMotorVoltage(frontRightAcc.speed)
            frontRightFFVoltage = frontRightFFVel + frontRightFFAcc

            backLeftFFVel = backLeftVel.speed / constants.physicalMaxSpeed * 12
            backLeftFFAcc = self.backLeftSwerveModule.accelerationToMotorVoltage(backLeftAcc.speed)
            backLeftFFVoltage = backLeftFFVel + backLeftFFAcc

            backRightFFVel = backRightVel.speed / constants.physicalMaxSpeed * 12
            backRightFFAcc = self.backRightSwerveModule.accelerationToMotorVoltage(backRightAcc.speed)
            backRightFFVoltage = backRightFFVel + backRightFFAcc

            self.frontLeftSwerveModule.setDesiredState(frontLeftVel, frontLeftFFVoltage)
            self.frontRightSwerveModule.setDesiredState(frontRightVel, frontRightFFVoltage)
            self.backLeftSwerveModule.setDesiredState(backLeftVel, backLeftFFVoltage)
            self.backRightSwerveModule.setDesiredState(backRightVel, backRightFFVoltage)

            self.autoVelocityChassisSpeedsTopic.set(self.autoVelocityChassisSpeeds)
            self.autoAccelerationChassisSpeedsTopic.set(self.autoAccelChassisSpeeds)
            self.frontLeftFFVelTopic.set(frontLeftFFVel)
            self.frontLeftFFAccTopic.set(frontLeftFFAcc)
            self.frontLeftFFTopic.set(frontLeftFFVoltage)
        else:
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


    

    def driveTeleop(
        self,
        xSpeed: wpimath.units.meters_per_second,
        ySpeed: wpimath.units.meters_per_second,
        turnSpeed: wpimath.units.radians_per_second,
        fieldRelative: bool,
    ):
        self.autonomousMode = False
        if fieldRelative:
            self.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                turnSpeed,
                self.odometry.getEstimatedPosition().rotation()
            )
        else:
            self.desiredChassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, turnSpeed)

    def driveAutonomous(
        self,
        xSpeed: wpimath.units.meters_per_second,
        ySpeed: wpimath.units.meters_per_second,
        omega: wpimath.units.radians_per_second,
        xAccel: wpimath.units.meters_per_second_squared,
        yAccel: wpimath.units.meters_per_second_squared,
        alpha: wpimath.units.radians_per_second_squared,
    ):
        self.autonomousMode = True
        self.autoVelocityChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            omega,
            self.odometry.getEstimatedPosition().rotation()
        )
        self.autoAccelChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xAccel,
            yAccel,
            alpha,
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

    def resetPose(self, pose: Pose2d):
        self.odometry.resetPose(pose)

    def setHeadingControllerMode(self, mode: SwerveHeadingMode):
        self.headingController.setMode(mode)

    def followChoreoSample(self, sample: choreo.trajectory.SwerveSample):
        pose = self.odometry.getEstimatedPosition()

        # self.driveTeleop(
        #     sample.vx + self.choreoXController.calculate(pose.X(), sample.x),
        #     sample.vy + self.choreoYController.calculate(pose.Y(), sample.y),
        #     sample.omega + self.choreoHeadingController.calculate(pose.rotation().radians(), sample.heading),
        #     fieldRelative= True
        # )

        # TODO: Eventually, add back PID :)
        self.driveAutonomous(sample.vx, sample.vy, sample.omega, sample.ax, sample.ay, sample.alpha)

    def setHeadingControllerGoal(self, angle : Rotation2d):
        self.headingController.setGoal(angle)
