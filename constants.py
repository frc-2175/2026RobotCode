import math
from wpimath.geometry import Translation2d, Transform3d
import wpimath.units


# Several values in this file are sourced from data sheets for our actual robot
# hardware:
# - REV NEO data sheet: https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf
# - REV 3in MAXSwerve product page: https://www.revrobotics.com/rev-21-3005/

robotMass = wpimath.units.lbsToKilograms(100)
"""Total mass of the entire robot."""

wheelDiameter = wpimath.units.inchesToMeters(3)
"""Diameter of a drive wheel."""

# TODO Verify that this number is accurate
wheelDistanceFromCenter = wpimath.units.inchesToMeters(11.875)
"""The distance along x or y from the center of the robot to a swerve wheel."""

swerveModulePositions = (
  Translation2d(wheelDistanceFromCenter, wheelDistanceFromCenter),
  Translation2d(wheelDistanceFromCenter, -wheelDistanceFromCenter),
  Translation2d(-wheelDistanceFromCenter, wheelDistanceFromCenter),
  Translation2d(-wheelDistanceFromCenter, -wheelDistanceFromCenter),
)
"""The positions of each swerve module relative to the robot origin, in the order FL/FR/BL/BR"""


driveMotorReduction = 4.71
""""High speed" gear ratio. Unit: ratio (N:1). (Source: REV MAXSwerve product page)"""

steerMotorReduction = 12
"""Gear ratio of the provided UltraPlanetary steering gearbox. (Source: REV MAXSwerve product page)"""

driveMotorFreeSpeed = 5676
"""Speed of a drive motor under no load. Unit: RPM. (Source: REV NEO data sheet)"""

physicalMaxSpeed: wpimath.units.meters_per_second = math.pi * wheelDiameter * driveMotorFreeSpeed / 60.0 / driveMotorReduction
"""Maximum possible speed of a single drive wheel. Unit: m/s."""

humanMaxSpeed = 4 #m/s
humanMaxTurnSpeed = 2 * math.pi #rad/s

headingControllerP = 1 / wpimath.units.degreesToRadians(15)
headingControllerI = 0
headingControllerD = 0

rotationSlewRate = 8 * math.pi #rad/s
maxAcceleration = 20 #m/s/s

intakeSpeed = 0.1
shooterSpeed = 0.65
rollerSpeed = 1
agitatorSpeed = 0.3
indexerSpeed = 1

bangBangTargetRPM = 3782
shotRPM = 3700

intakeMotorReduction = 20
shooterMotorRatio = 24/16

intakeOutAngle = wpimath.units.degreesToRadians(-10)
intakeInAngle = wpimath.units.degreesToRadians(-106)

choreoTranslationP = 2/wpimath.units.inchesToMeters(20)
choreoTranslationI = 0
choreoTranslationD = 6/wpimath.units.feetToMeters(20)
choreoRotationP = 1/wpimath.units.degreesToRadians(30)
choreoRotationI = 0
choreoRotationD = 1/wpimath.units.degreesToRadians(75)

robotToCam = Transform3d()
