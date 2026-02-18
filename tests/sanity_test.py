import math
from wpimath.geometry import Rotation2d
import wpimath.units

from testutil import expecteq, expectclose

# Can WPILib write code?

def d2r(d: wpimath.units.degrees) -> wpimath.units.radians:
    return wpimath.units.degreesToRadians(d)

expecteq(Rotation2d(math.pi).radians(), math.pi)
expecteq(Rotation2d(math.pi).degrees(), 180)

expecteq(Rotation2d(0), Rotation2d(0))
expecteq(Rotation2d(0), Rotation2d(2 * math.pi))
expecteq(Rotation2d(0), Rotation2d(4 * math.pi))

expecteq(Rotation2d(0) - Rotation2d(2 * math.pi), Rotation2d(0))
expecteq(Rotation2d(0) - Rotation2d(2 * math.pi), Rotation2d(2 * math.pi))
expecteq(Rotation2d(0) - Rotation2d(2 * math.pi), Rotation2d(4 * math.pi))

expecteq((Rotation2d(0) - Rotation2d(0)).radians(), 0)
expectclose((Rotation2d(0) - Rotation2d(2 * math.pi)).radians(), 0, 1e-9)
expectclose((Rotation2d(0) - Rotation2d(4 * math.pi)).radians(), 0, 1e-9)

expectclose((Rotation2d(d2r(10)) - Rotation2d(d2r(350))).degrees(), 20, 1e-9)
expectclose((Rotation2d(d2r(350)) - Rotation2d(d2r(10))).degrees(), -20, 1e-9) # I really don't know why this is -20.
