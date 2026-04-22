import wpimath.units
from typing import Generic, TypeVar
import math

from utils.mathutil import Vector2d

Unit = TypeVar("Unit", bound=float)

class SlewRateLimiter2D(Generic[Unit]):
    def __init__(self, rateLimit: wpimath.units.units_per_second, initialValue = Vector2d[Unit]()):
        self.rateLimit = rateLimit
        self.lastOutput = initialValue

    def calculate(self, target: Vector2d[Unit]) -> Vector2d[Unit]:
        x :Vector2d = target - self.lastOutput
        xLength = math.sqrt(x.x**2 + x.y**2)
        if xLength < self.rateLimit:
            r: Vector2d = x
        else:
            r: Vector2d = x * (self.rateLimit / xLength)
        self.lastOutput = self.lastOutput + r
        return self.lastOutput
