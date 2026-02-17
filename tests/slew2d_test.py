from testutil import expecteq
from utils.mathutil import Vector2d
from utils.slew2d import SlewRateLimiter2D


limiter = SlewRateLimiter2D(0.5, initialValue=Vector2d(0.0, 0.0))

# Slowly move to [4, 0]
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(0.5, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(1, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(1.5, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(2, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(2.5, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(3, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(3.5, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(4, 0))

# Make two very small adjustments
expecteq(limiter.calculate(Vector2d(4.1, 0)), Vector2d(4.1, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(4, 0))

# Stay at [4, 0]
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(4, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(4, 0))
expecteq(limiter.calculate(Vector2d(4, 0)), Vector2d(4, 0))

# Move to [4, 4]
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 0.5))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 1))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 1.5))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 2))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 2.5))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 3))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 3.5))

# Stay at [4, 4]
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 4))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 4))
expecteq(limiter.calculate(Vector2d(4, 4)), Vector2d(4, 4))
