import wpimath.units
from typing import Generic, TypeVar

from utils.mathutil import Vector2d

Unit = TypeVar("Unit", bound=float)

class SlewRateLimiter2D(Generic[Unit]):
    def __init__(self, rateLimit: wpimath.units.units_per_second, initialValue = Vector2d[Unit]()):
        self.rateLimit = rateLimit
        self.lastOutput = initialValue

    def calculate(self, target: Vector2d[Unit]) -> Vector2d[Unit]:
        # The general shape of what should be implemented here:
        # - Compute that "r" vector we talked about yesterday - a vector pointing
        #   from self.lastOutput to target that is scaled down to a maximum length,
        #   defined by self.rateLimit.
        # - Add that to lastOutput to get the new, slewed output.
        # - Save that to lastOutput.
        # - Return that value to the outer code.
        # There are automated tests for this that you can run.
        pass
