import rev
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
import wpimath.units

import configs


class SwerveModule:
    """
    The SwerveModule class contains common logic for controlling a swerve
    module. While we could control all eight motors directly from the
    Drivetrain subsystem, this would be repetitive and error-prone.

    Each swerve module has a drive motor and a steer motor, as well as an
    angleOffset which will be added to the raw value from the absolute encoder.
    This is to compensate for the fact that each swerve module is physically
    rotated differently on the robot, yet are all calibrated the same way.
    """
    
    def __init__(self, driveMotorId: int, steerMotorId: int, angleOffset: wpimath.units.radians):
        """
        :param driveMotorId: The ID of the SPARK MAX for the drive motor.
        :param steerMotorId: The ID of the SPARK MAX for the steering motor.
        :param angleOffset: A value, in radians, to be added to the absolute
               encoder value. This should be chosen such that the encoder +
               `angleOffset` reads zero when the wheel is pointing straight
               forward (zero rotation in robot coordinates).
        """
        self.driveMotor = rev.SparkMax(driveMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.steerMotor = rev.SparkMax(steerMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.angleOffset = angleOffset

        self.driveMotor.configure(configs.driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.steerMotor.configure(configs.steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.steerEncoder = self.steerMotor.getAbsoluteEncoder()

        self.drivePidController = self.driveMotor.getClosedLoopController()
        self.steerPidController = self.steerMotor.getClosedLoopController()

    def setDesiredState(self, state: SwerveModuleState):
        """
        Sets the desired state of the swerve module (angle/speed). This method
        will account for the swerve module's angle offset, so the angle
        provided should NOT be modified to account for angle offsets. (In other
        words, pass the raw SwerveModuleState straight out of kinematics.)
        """
        correctedAngle = state.angle - Rotation2d(self.angleOffset)
        # In real robot projects, we might do other work here to improve swerve
        # behavior, e.g. driving wheels more slowly when they are still pointing in
        # the wrong direction.
        self.drivePidController.setReference(state.speed, rev.SparkLowLevel.ControlType.kVelocity)
        self.steerPidController.setReference(correctedAngle.radians(), rev.SparkLowLevel.ControlType.kPosition)

    def getActualState(self) -> SwerveModuleState:
        """
        Gets the actual state of the swerve module (angle/speed).
        """
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.steerEncoder.getPosition() + self.angleOffset)
        )

    def getActualPosition(self) -> SwerveModulePosition:
        """
        Gets the actual position of the swerve module (drive/steer positions).
        """
        return SwerveModulePosition(
            self.driveEncoder.getPosition(),
            Rotation2d(self.steerEncoder.getPosition() + self.angleOffset)
        )
