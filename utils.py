from __future__ import annotations 
from typing import Generic, SupportsInt, TypeVar
import typing
from wpimath.geometry import Rotation2d, Translation2d
import wpimath.units


# =============================================================================
# This file contains simple utilities that are generally useful throughout all
# our Python code.
# =============================================================================

def lerp(a: float, b: float, t: float) -> float:
    """
    "Lerp" is short for "Linear intERPolation". It can be used to interpolate
    (blend) between two values, `a` and `b`, by a factor of `t`. Examples:

        lerp(10, 20, t=0)   => 10
        lerp(10, 20, t=1)   => 20
        lerp(10, 20, t=0.5) => 15  # halfway between a and b

    """
    return (1-t) * a + t * b

def clamp(value: float, min: float, max: float) -> float:
    """
    Limits the input `value` to the given `min` and `max`, useful for enforcing
    a safe range of values. Examples:

        clamp(-10, min=-1, max=1) => -1   # limited to min
        clamp(10, min=-1, max=1)  => 1    # limited to max
        clamp(0.5, min=-1, max=1) => 0.5  # unchanged

    """
    assert min <= max, f"in utils.clamp: min ({min}) and max ({max}) are reversed"
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value

def remap(value: float, before: tuple[float, float], after: tuple[float, float]) -> float:
    """
    Takes two ranges of values, `before` and `after`, and maps the input
    `value` from `before` to `after`. This is often useful for joystick inputs,
    where for example you may want -1 and 1 to correspond to your minimum and
    maximum robot speeds. Examples:

        remap(  1, (-1, 1), (-math.pi, math.pi)) => math.pi
        remap( -1, (-1, 1), (-math.pi, math.pi)) => -math.pi
        remap(0.5, (-1, 1), (-math.pi, math.pi)) => math.pi/2

    """
    t = (value - before[0]) / (before[1] - before[0])
    return lerp(after[0], after[1], t)

def sign(v: float) -> float:
    """
    Extracts the sign of a number: -1 for negative numbers, 1 for positive
    numbers and zero.
    """
    if v < 0:
        return -1
    else:
        return 1

def sign_or_zero(v: float) -> float:
    """
    Extracts the sign of a number: -1 for negative numbers, 1 for positive
    numbers, and 0 for zero.
    """
    if v == 0:
        return 0
    else:
        return sign(v)

Unit = TypeVar("Unit", bound=float, default=float)

class Vector2d(Generic[Unit]):
    """
    A generic wrapper around WPILib's Translation2d that allows the user to
    parameterize the unit type (default Translation2d uses meters).
    """

    @typing.overload
    def __init__(self):
        """
        Constructs a Vector2d with X and Y components equal to zero.
        """
        ...

    @typing.overload
    def __init__(self, x: Unit, y: Unit):
        """
        Constructs a Vector2d with the X and Y components equal to the
        provided values.
        
        :param x: The x component of the vector.
        :param y: The y component of the vector.
        """
        ...

    def __init__(self, arg0 = None, arg1 = None): # type: ignore
        if arg0 is None and arg1 is None:
            self.translation = Translation2d()
        else:
            self.translation = Translation2d(arg0, arg1) # type: ignore

    @classmethod
    def fromTranslationRaw(cls, t: Translation2d):
        return cls(t.X(), t.Y()) # type: ignore

    @staticmethod
    def fromTranslation(t: Translation2d) -> Vector2d[wpimath.units.meters]:
        return Vector2d[wpimath.units.meters].fromTranslationRaw(t)

    @classmethod
    def fromMagnitudeAndDirection(cls, m: Unit, dir: Rotation2d) -> Vector2d[Unit]:
        return cls(m * dir.cos(), m * dir.sin()) # type: ignore

    def toTranslation(self) -> Translation2d:
        """Return the underlying WPILib Translation2d object."""
        return self.translation

    def X(self) -> Unit:
        """
        Returns the X component of the vector.
        
        :returns: The X component of the vector.
        """
        return self.translation.X() # type: ignore
    
    def Y(self) -> Unit:
        """
        Returns the Y component of the vector.
        
        :returns: The Y component of the vector.
        """
        return self.translation.Y() # type: ignore

    def __abs__(self) -> Unit:
        return self.translation.__abs__() # type: ignore

    def __add__(self, other: Vector2d[Unit]) -> Vector2d[Unit]:
        """
        Returns the sum of two vectors in 2D space.
        
        For example, Vector2d(1.0, 2.5) + Vector2d(2.0, 5.5) =
        Vector2d(3.0, 8.0).
        
        :param other: The vector to add.
        
        :returns: The sum of the vectors.
        """
        return Vector2d[Unit].fromTranslationRaw(self.translation.__add__(other.translation))

    def __eq__(self, other: object) -> bool:
        """
        Checks equality between this Vector2d and another object.
        
        :param other: The other object.
        
        :returns: Whether the two objects are equal.
        """
        if not isinstance(other, Vector2d):
            return False
        return self.translation == other.translation

    def __getitem__(self, arg0: SupportsInt) -> Unit:
        return self.translation.__getitem__(arg0) # type: ignore

    def __len__(self) -> int:
        return self.translation.__len__()
    
    def __mul__(self, scalar: typing.SupportsFloat) -> Vector2d[Unit]:
        """
        Returns the vector multiplied by a scalar.
        
        For example, Vector2d(2.0, 2.5) * 2 = Vector2d(4.0, 5.0).
        
        :param scalar: The scalar to multiply by.
        
        :returns: The scaled translation.
        """
        return Vector2d[Unit].fromTranslationRaw(self.translation.__mul__(scalar))

    def __neg__(self) -> Vector2d[Unit]:
        """
        Returns the inverse of the current vector. This is equivalent to
        rotating by 180 degrees, flipping the point over both axes, or negating all
        components of the vector.
        
        :returns: The inverse of the current vector.
        """
        return Vector2d[Unit].fromTranslationRaw(self.translation.__neg__())

    def __repr__(self) -> str:
        return self.translation.__repr__()

    def __sub__(self, other: Vector2d[Unit]) -> Vector2d[Unit]:
        """
        Returns the difference between two vectors.
        
        For example, Vector2d(5.0, 4.0) - Vector2d(1.0, 2.0) =
        Vector2d(4.0, 2.0).
        
        :param other: The vector to subtract.
        
        :returns: The difference between the two vectors.
        """
        return Vector2d[Unit].fromTranslationRaw(self.translation.__sub__(other.translation))

    def __truediv__(self, scalar: typing.SupportsFloat) -> Vector2d[Unit]:
        """
        Returns the vector divided by a scalar.
        
        For example, Vector2d(2.0, 2.5) / 2 = Vector2d(1.0, 1.25).
        
        :param scalar: The scalar to divide by.
        
        :returns: The scaled vector.
        """
        return Vector2d[Unit].fromTranslationRaw(self.translation.__truediv__(scalar))
    
    def angle(self) -> Rotation2d:
        """
        Returns the angle this vector forms with the positive X axis.
        
        :returns: The angle of the vector
        """
        # WPILib screams at you in this extremely obvious case.
        if self.x == 0 and self.y == 0:
            return Rotation2d()
        return self.translation.angle()

    def norm(self) -> Unit:
        """
        Returns the norm, or magnitude of the vector.
        
        :returns: The norm of the vector.
        """
        return self.translation.norm() # type: ignore

    def normalized(self) -> Vector2d[Unit]:
        return Vector2d[Unit].fromMagnitudeAndDirection(1, self.angle()) # type: ignore

    def rotateAround(self, other: Vector2d[Unit], rot: Rotation2d) -> Vector2d[Unit]:
        """
        Rotates this vector around another vector in 2D space.
        
        ::
        
          [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
          [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
        
        :param other: The other vector to rotate around.
        :param rot:   The rotation to rotate the vector by.
        
        :returns: The new rotated vector.
        """
        return Vector2d[Unit].fromTranslationRaw(self.translation.rotateAround(other.translation, rot))
    
    def rotateBy(self, rot: Rotation2d) -> Vector2d[Unit]:
        """
        Applies a rotation to the vector in 2D space.
        
        This multiplies the vector by a counterclockwise rotation
        matrix of the given angle.
        
        ::
        
          [x_new]   [other.cos, -other.sin][x]
          [y_new] = [other.sin,  other.cos][y]
        
        For example, rotating a Vector2d of &lt;2, 0&gt; by 90 degrees will
        return a Vector2d of &lt;0, 2&gt;.
        
        :param other: The rotation to rotate the vector by.
        
        :returns: The new rotated vector.
        """
        return Vector2d[Unit].fromTranslationRaw(self.translation.rotateBy(rot))

    @property
    def x(self) -> Unit:
        return self.X()

    @property
    def y(self) -> Unit:
        return self.Y()

    # Now that we are not encumbered by the idea of "translation", we can
    # actually have real vector operations, as we ought.

    def dot(self, other: Vector2d) -> float:
        """
        Calculates the dot product of two Vector2ds.

        :return: The dot product of the two vectors.
        """
        return (self.x * other.x) + (self.y * other.y)

    def cross(self, other: Vector2d) -> float:
        """
        Calculates the pseudo-cross-product of two Vector2ds. This operation is
        typically defined only in three dimensions, but by interpreting the
        vectors as 3D with z = 0, the `(a.x * b.y) - (a.y * b.x)` quantity is
        the natural result.

        The resulting unit will be the product of the two input units. For
        example, crossing a vector in Newtons with a vector in meters will
        produce Newton-meters.
        
        :return: The "cross product" of the two vectors.
        """
        return (self.x * other.y) - (self.y * other.x) # type: ignore
