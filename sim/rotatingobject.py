import wpimath.units

import ntutil
import utils


class RotatingObject:
    """
    A class used to simulate the physics of a rotating object with inertia,
    drag, and friction.
    """

    def __init__(
        self,
        *, momentOfInertia: wpimath.units.kilogram_square_meters,
        viscousDragCoefficient = 0.001, # in Nm/(rad/s) (Newton-meters per (radian per second))
        friction: wpimath.units.newton_meters = 0.01,
        nt: ntutil.Folder = ntutil.DummyFolder()
    ):
        """
        :param momentOfInertia: The object's moment of inertia. This can be
               calculated using the `moi*` functions found in this file.
        :param viscousDragCoefficient: Coefficient of friction that scales with
               velocity, like air drag. The unit is Nm/(rad/s) (Newton-meters
               per (radian per second)).
        :param friction: A constant torque opposing motion, simulating
               friction. The unit is Nm (Newton-meters).
        :param nt: A NetworkTables folder to use for logging information.
        """
        self.moi = momentOfInertia
        self.viscousDrag = viscousDragCoefficient
        self.friction = friction

        self.velocity: wpimath.units.radians_per_second = 0

        self.inputTorqueTopic = nt.getFloatTopic("InputTorque")
        self.viscousTorqueTopic = nt.getFloatTopic("ViscousDragTorque")
        self.frictionTorqueTopic = nt.getFloatTopic("FrictionTorque")
        self.totalTorqueTopic = nt.getFloatTopic("TotalTorque")
        self.velocityTopic = nt.getFloatTopic("Velocity")

    def iterate(self, inputTorque: wpimath.units.newton_meters, dt: float):
        # Compute total torque (output plus drag / friction)
        viscousDragTorque = self.viscousDrag * -self.velocity
        frictionTorque = utils.sign_or_zero(-self.velocity) * self.friction # already Nm
        totalTorque = inputTorque + viscousDragTorque + frictionTorque

        # Torque -> accleration -> updated velocity.
        #
        # Since friction is a constant force, naively applying it at low speeds
        # can cause us to reverse direction. We hack around this problem with a
        # simple rule: the velocity can never change sign in a single tick. If
        # this happens, we just set the velocity to zero. If there is still a
        # torque applied, it will take effect one tick late.
        angularAcceleration = totalTorque / self.moi # rad/(s^2)
        newVelocity = self.velocity + (angularAcceleration * dt)
        if self.velocity != 0 and utils.sign(newVelocity) != utils.sign(self.velocity):
            self.velocity = 0
        else:
            self.velocity = newVelocity

        self.inputTorqueTopic.set(inputTorque)
        self.viscousTorqueTopic.set(viscousDragTorque)
        self.frictionTorqueTopic.set(frictionTorque)
        self.totalTorqueTopic.set(totalTorque)
        self.velocityTopic.set(self.velocity)
    
    def getVelocity(self) -> wpimath.units.radians_per_second:
        return self.velocity


def moiWheel(
    mass: wpimath.units.kilograms,
    outerDiameter: wpimath.units.meters,
    innerDiameter: wpimath.units.meters,
) -> wpimath.units.kilogram_square_meters:
    """
    Computes the moment of inertia for a wheel.
    """
    r1 = innerDiameter / 2
    r2 = outerDiameter / 2
    return 0.5 * mass * (r2*r2 + r1*r1)


def moiArm(
    mass: wpimath.units.kilograms,
    centerOfMassRadius: wpimath.units.meters,
) -> wpimath.units.kilogram_square_meters:
    """
    Computes the moment of inertia for an arm or other point mass.
    """
    return mass * centerOfMassRadius * centerOfMassRadius
