import math
from wpimath.geometry import Translation2d
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

# TODO: Verify the gear ratio used on this year's bot
driveMotorReduction = 4.71
""""High speed" gear ratio. Unit: ratio (N:1). (Source: REV MAXSwerve product page)"""

steerMotorReduction = 12
"""Gear ratio of the provided UltraPlanetary steering gearbox. (Source: REV MAXSwerve product page)"""

driveMotorFreeSpeed = 5676
"""Speed of a drive motor under no load. Unit: RPM. (Source: REV NEO data sheet)"""

maxSpeed = math.pi * wheelDiameter * driveMotorFreeSpeed / 60.0 / driveMotorReduction
"""Maximum possible speed of a single drive wheel. Unit: m/s."""
