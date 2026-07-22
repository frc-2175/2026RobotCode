import rev
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
import wpimath.units
import constants

import configs
from utils import mathutil

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

        self.driveMotor.configure(configs.driveMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.steerMotor.configure(configs.steerMotorConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.steerEncoder = self.steerMotor.getAbsoluteEncoder()

        self.drivePidController = self.driveMotor.getClosedLoopController()
        self.steerPidController = self.steerMotor.getClosedLoopController()

    def setDesiredState(self, state: SwerveModuleState, raw_torque: wpimath.units.newton_meters | None = None):
        """
        Sets the desired state of the swerve module (angle/speed). This method
        will account for the swerve module's angle offset, so the angle
        provided should NOT be modified to account for angle offsets. (In other
        words, pass the raw SwerveModuleState straight out of kinematics.)
        """
        stateLocal = SwerveModuleState(state.speed, Rotation2d(state.angle.radians() - self.angleOffset))
        encoderRotation = Rotation2d(self.steerEncoder.getPosition())
        # stateLocal.optimize(encoderRotation)
        stateLocal.cosineScale(encoderRotation)
        if raw_torque is None:
            self.drivePidController.setSetpoint(stateLocal.speed, rev.SparkLowLevel.ControlType.kVelocity)
        else:
            self.driveMotor.setVoltage(self.torqueToOutputVoltage(raw_torque))
        self.steerPidController.setSetpoint(stateLocal.angle.radians(), rev.SparkLowLevel.ControlType.kPosition)

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

    # Function to compute output for a desired torque, based on current wheel speed,
    # according to the data sheet: https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf
    def torqueToOutputVoltage(self, torque: wpimath.units.newton_meters) -> wpimath.units.volts:
        max_torque = 2.6 # Nm
        max_rpm = 5676 # rpm at which we get 0 torque
        max_volts = 12
        current_rpm = self.driveEncoder.getVelocity() / self.driveMotor.configAccessor.encoder.getVelocityConversionFactor()
        current_max_torque = mathutil.lerp(max_torque, 0, current_rpm / max_rpm)
        output_fraction = torque / current_max_torque
        output_volts = output_fraction * max_volts
        return output_volts

    def accelerationToMotorTorque(self, accel: wpimath.units.meters_per_second_squared) -> wpimath.units.newton_meters:
        motorTorque = (accel * (constants.wheelDiameter/2) * constants.robotMass) / 4.71
        return motorTorque
