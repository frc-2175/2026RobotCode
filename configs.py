import math
import rev
from wpilib import RobotBase

import constants


# =============================================================================
# This file contains configuration objects for our motors. We put them all here
# because they are big and bulky and more analogous to the constants we have in
# constants.py.
# =============================================================================

driveMotorConfig = rev.SparkMaxConfig()
driveMotorConfig.smartCurrentLimit(40)
driveMotorConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
# Convert position from rotations to m
driveMotorConfig.encoder.positionConversionFactor(math.pi * constants.wheelDiameter / constants.driveMotorReduction)
# Convert velocity from RPM to m/s
driveMotorConfig.encoder.velocityConversionFactor(
    math.pi * constants.wheelDiameter / constants.driveMotorReduction / 60
)
# Use PIDF for control with the built-in NEO encoder for feedback.
driveMotorConfig.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
driveMotorConfig.closedLoop.pidf(0, 0, 0, 1 / constants.maxSpeed)

steerMotorConfig = rev.SparkMaxConfig()
steerMotorConfig.smartCurrentLimit(40)
# Convert velocity (as measured by built-in relative encoder) from RPM to rad/s.
# TODO: Shouldn't this take the gear ratio of the steering gearbox into account?
steerMotorConfig.encoder.velocityConversionFactor(2 * math.pi / 60)
# Invert the absolute encoder so counter-clockwise is positive.
steerMotorConfig.absoluteEncoder.inverted(True)
# Convert position (as measured by absolute encoder) from rotations to radians.
steerMotorConfig.absoluteEncoder.positionConversionFactor(2 * math.pi)
# Convert velocity (as measured by absolute encoder) from RPM to rad/s.
steerMotorConfig.absoluteEncoder.velocityConversionFactor(2 * math.pi / 60)
# Use PID for control with the absolute encoder for feedback. Because this PID
# controller works in angles, which wrap around, we enable position wrapping.
steerMotorConfig.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)
steerMotorConfig.closedLoop.pid(1, 0, 0.1)
steerMotorConfig.closedLoop.positionWrappingEnabled(True)
steerMotorConfig.closedLoop.positionWrappingInputRange(-math.pi, math.pi)
