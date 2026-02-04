import math
import random
import typing

import hal.simulation
import pyfrc.physics.core
from pyfrc.physics.core import PhysicsInterface
import rev
from rev import SparkMax, SparkMaxSim
from wpilib import RobotController
from wpilib.simulation import BatterySim, RoboRioSim
import wpimath
import wpimath.units
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.system.plant import DCMotor

import constants
from hardware.swervemodule import SwerveModule
import ntutil
from robot import MyRobot
from sim.rotatingobject import RotatingObject, moiWheel
from subsystems.drivetrain import Drivetrain
import utils
from utils import Vector2d


# =============================================================================
# This file controls the physics simulation used when running in the WPILib
# simulator. You should NOT need to touch it for the homework.
# =============================================================================

# See https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
class PhysicsEngine(pyfrc.physics.core.PhysicsEngine):
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # NetworkTables topics
        simFolder = ntutil.getFolder("Sim")
        self.poseTopic = simFolder.getStructTopic("Pose", Pose2d)
        self.vbusTopic = simFolder.getFloatTopic("VBus")

        # Physics device setup
        self.robot = robot
        # TODO: Our current calculations are currently returning abject
        # nonsense. For now we will just bypass current draw simulation.
        # self.battery = DrainingBatterySim(nt=simFolder.folder("Battery"))
        self.battery = NormalBatterySim(nt=simFolder.getFolder("Battery"))
        self.drivetrain = DrivetrainSim(
            robot.drivetrain,
            mass=constants.robotMass,
            slipFriction=128,
            nt=simFolder.getFolder("Drivetrain"),
        )

    def update_sim(self, now: float, tm_diff: float):
        vbus = RobotController.getBatteryVoltage()
        self.vbusTopic.set(vbus)

        self.drivetrain.iterate(vbus, tm_diff)

        self.poseTopic.set(self.drivetrain.getPose())
        # self.swerveStatesTopic.set(list(self.drivetrain.get_module_states()))

        # Update simulated electrical state
        currents = [self.drivetrain.getCurrentDraw()]
        self.battery.iterate(sum(currents), tm_diff)
        RoboRioSim.setVInVoltage(self.battery.outputVoltage())
        RoboRioSim.setVInCurrent(self.battery.outputCurrent())


class DrivetrainSim:
    def __init__(
        self,
        drivetrain: Drivetrain,
        *, mass: wpimath.units.kilograms,
        slipFriction: wpimath.units.newtons = 0,
        nt: ntutil.Folder = ntutil.DummyFolder(),
    ):
        self.realDrivetrain = drivetrain
        self.modules: tuple[SwerveModuleSim, SwerveModuleSim, SwerveModuleSim, SwerveModuleSim] = (
            SwerveModuleSim(drivetrain.frontLeftSwerveModule, nt=nt.getFolder("FrontLeft")),
            SwerveModuleSim(drivetrain.frontRightSwerveModule, nt=nt.getFolder("FrontRight")),
            SwerveModuleSim(drivetrain.backLeftSwerveModule, nt=nt.getFolder("BackLeft")),
            SwerveModuleSim(drivetrain.backRightSwerveModule, nt=nt.getFolder("BackRight")),
        )
        self.kinematics = drivetrain.kinematics

        # Per the navX docs, the recommended way to use the navX as a sim
        # device is to just use the real device but set the simulator variable
        # for Yaw directly.
        #
        # https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/c/
        navXDevice = hal.simulation.getSimDeviceHandle(f"navX-Sensor[{drivetrain.gyro.getPort()}]")
        self.navXAngleSim = hal.SimDouble(hal.simulation.getSimValueHandle(navXDevice, "Yaw"))

        self.pose = Pose2d(Translation2d(10, 5), Rotation2d())
        self.mass = mass
        self.moi = moiWheel(mass, wpimath.units.inchesToMeters(34), 0) # assume the robot is a flat disk...
        self.slipFriction = slipFriction
        self.velocity = Vector2d[wpimath.units.meters_per_second]()
        self.angularVelocity: wpimath.units.radians_per_second = 0
        self.moduleFieldVelocities: list[Vector2d[wpimath.units.meters_per_second]] = [Vector2d(), Vector2d(), Vector2d(), Vector2d()]

        self.velocityVisualTopic = nt.getStructTopic("VelocityVisual", ChassisSpeeds)
        self.netForceTopic = nt.getStructTopic("NetForce", Translation2d)
        self.netForceVisualTopic = nt.getStructTopic("NetForceVisual", Translation2d)
        self.netTorqueTopic = nt.getFloatTopic("NetTorque")
        self.moduleFieldAnglesVisualTopic = nt.getStructArrayTopic("ModuleFieldAnglesVisual", SwerveModuleState)
        self.moduleVelocitiesTopic = nt.getStructArrayTopic("ModuleVelocities", Translation2d)
        self.moduleVelocitiesVisualTopic = nt.getStructArrayTopic("ModuleVelocitiesVisual", SwerveModuleState)
        self.moduleForcesTopic = nt.getStructArrayTopic("ModuleForces", Translation2d)
        self.moduleForcesVisualTopic = nt.getStructArrayTopic("ModuleForcesVisual", SwerveModuleState)
        self.moduleDragForcesTopic = nt.getStructArrayTopic("ModuleDragForces", Translation2d)
        self.moduleDragForcesVisualTopic = nt.getStructArrayTopic("ModuleDragForcesVisual", SwerveModuleState)

    def iterate(self, vbus: float, dt: float):
        modulePosesBefore = self.getModuleFieldPoses()

        for module, moduleVelocity in zip(self.modules, self.moduleFieldVelocities):
            moduleVelocityRobot = moduleVelocity.rotateBy(-self.pose.rotation())
            module.iterate(moduleVelocityRobot, vbus, dt)
        moduleStates = self.getModuleStates()

        moduleFieldAngles = [self.pose.rotation() + m.angle for m in moduleStates]        
        moduleDriveForces = [Vector2d.fromMagnitudeAndDirection(m.getDriveForce(), angle) for m, angle in zip(self.modules, moduleFieldAngles)]
        moduleDragForces: list[Vector2d[wpimath.units.newtons]] = []
        for angle, velocity in zip(moduleFieldAngles, self.moduleFieldVelocities):
            direction = Vector2d.fromMagnitudeAndDirection(1, angle)
            alignmentFactor = utils.clamp(utils.remap(
                direction.dot(velocity.normalized()),
                (1, math.cos(wpimath.units.degreesToRadians(30))),
                (0, 1),
            ), 0, 1)
            velocityFactor = utils.remap(velocity.norm(), (0, 0.5), (0, 1))
            dragForce = utils.lerp(0, self.slipFriction, min(alignmentFactor, velocityFactor))
            moduleDragForces.append(-Vector2d.fromMagnitudeAndDirection(dragForce, velocity.angle()))

        # Compute net force and torque on the robot by applying all four forces
        # at the appropriate offsets. See the following link (sec. 2.7) for more
        # background: https://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf
        netForce = Vector2d[wpimath.units.newtons]()
        netTorque: wpimath.units.newton_meters = 0
        for fv, fd, modPose in zip(moduleDriveForces, moduleDragForces, modulePosesBefore):
            modOffsetField = Vector2d.fromTranslation(modPose.translation() - self.pose.translation())
            netForce += fv
            netForce += fd
            netTorque += modOffsetField.cross(fv)
            netTorque += modOffsetField.cross(fd)

        accleration: Vector2d[wpimath.units.meters_per_second_squared] = netForce / self.mass
        angularAcceleration = netTorque / self.moi

        self.velocity += accleration * dt
        self.angularVelocity += angularAcceleration * dt
        newPosition = self.pose.translation() + (self.velocity * dt).toTranslation()
        newRotation = self.pose.rotation().rotateBy(Rotation2d(self.angularVelocity * dt))

        self.pose = Pose2d(newPosition, newRotation)
        
        modulePosesAfter = self.getModuleFieldPoses()
        self.moduleFieldVelocities = [
            Vector2d.fromTranslation(after.translation() - before.translation()) / dt
            for before, after in zip(modulePosesBefore, modulePosesAfter)
        ]

        # NetworkTables values
        self.velocityVisualTopic.set(ChassisSpeeds(self.velocity.x, self.velocity.y, self.angularVelocity))
        self.moduleFieldAnglesVisualTopic.set([
            SwerveModuleState(1, angle - self.pose.rotation()) for angle in moduleFieldAngles
        ])
        self.moduleForcesTopic.set([fv.toTranslation() for fv in moduleDriveForces])
        self.moduleForcesVisualTopic.set([
            SwerveModuleState(fv.norm() / 10, fv.angle() - self.pose.rotation())
            for fv in moduleDriveForces
        ])
        self.moduleDragForcesTopic.set([d.toTranslation() for d in moduleDragForces])
        self.moduleDragForcesVisualTopic.set([
            SwerveModuleState(df.norm() / 10, df.angle() - self.pose.rotation())
            for df in moduleDragForces
        ])
        self.moduleVelocitiesTopic.set([v.toTranslation() for v in self.moduleFieldVelocities])
        self.moduleVelocitiesVisualTopic.set([
            SwerveModuleState(v.norm(), v.angle() - self.pose.rotation())
            for v in self.moduleFieldVelocities
        ])
        self.netForceTopic.set(netForce.toTranslation())
        self.netTorqueTopic.set(netTorque)
        self.netForceVisualTopic.set(self.pose.translation() + (netForce.toTranslation() / 10))

        # Update the simulated gyro. For reasons unknown, the navX uses
        # clockwise degrees.
        self.navXAngleSim.set(-self.pose.rotation().degrees())

    def getPose(self):
        return self.pose

    def setPose(self, pose: Pose2d):
        self.pose = pose

    def getModuleStates(self):
        return [m.getState() for m in self.modules]

    def getModuleFieldPoses(self):
        return [self.pose.transformBy(Transform2d(t, Rotation2d())) for t in constants.swerveModulePositions]

    def getCurrentDraw(self) -> wpimath.units.amperes:
        current = sum(s.getCurrentDraw() for s in self.modules)
        return current


class SwerveModuleSim:
    """
    Simulates a single swerve module.

    Velocities of each motor are computed like so:
     - Steer motor velocity is simulated using "flywheel"-style physics sim
       with significant friction. It is assumed that the steer motor's velocity
       conversion factor is to radians per second.
     - Drive motor velocity is simulated using the motion of the simulated
       swerve module (that is, its linear motion from tick to tick). This
       requires input from outside. It is assumed that the drive motor's
       velocity conversion factor is to meters per second.
    """
    def __init__(
        self,
        module: SwerveModule,
        *, nt: ntutil.Folder = ntutil.DummyFolder(),
    ):
        self.realModule = module
        self.driveSparkSim = SparkMaxSim2175(
            module.driveMotor,
            DCMotor.NEO(1),
            gearReduction=constants.driveMotorReduction,
            nt=nt.getFolder("DriveMotor"),
        )
        self.steerSparkSim = SparkMaxSim2175(
            module.steerMotor,
            DCMotor.NEO550(1),
            gearReduction=constants.steerMotorReduction,
            nt=nt.getFolder("SteerMotor"),
        )

        self.steerPhysics = RotatingObject(
            # Pretty random estimates for the mass of the rotating part and the
            # "diameters" of this definitely not wheel-shaped thing, but it
            # shouldn't really matter.
            momentOfInertia=moiWheel(
                mass=2,
                outerDiameter=wpimath.units.inchesToMeters(5),
                innerDiameter=wpimath.units.inchesToMeters(0.5),
            ),

            # Rough estimate for scrub friction (from ChatGPT with number
            # ranges sanity-checked): T = kF * N * A, where T = friction
            # torque (Newton-meters), kF = coefficient of friction, N = normal
            # force (Newtons), A = contact area (meters^2). Plausible values
            # for all result in plausible-enough results, I guess?
            # friction=0.3 * 110 * 0.05,

            # Or we could just throw in a number that feels like it works.
            friction=6,

            nt=nt.getFolder("SteerPhysics"),
        )

        # Randomize the starting rotation to simulate what we have when we take
        # the field
        self.steerSparkSim.setPosition(random.uniform(-math.pi, math.pi))

    def iterate(
        self,
        moduleVelocityRobot: Vector2d[wpimath.units.meters_per_second],
        vbus: float,
        dt: float,
    ):
        # Iterate motor controllers
        driveForwardDir = Vector2d.fromMagnitudeAndDirection(1, self.getState().angle)
        driveVelocity: wpimath.units.meters_per_second = max(0, moduleVelocityRobot.dot(driveForwardDir))
        self.driveSparkSim.iterate(driveVelocity, vbus, dt)
        self.steerSparkSim.iterate(self.steerPhysics.getVelocity(), vbus, dt)

        # Update steer physics for next tick
        self.steerPhysics.iterate(self.steerSparkSim.getMotorTorque(), dt)

    def getState(self) -> SwerveModuleState:
        return self.realModule.getActualState()
    
    def getDriveForce(self) -> wpimath.units.newtons:
        return self.driveSparkSim.getMotorTorque() / (constants.wheelDiameter / 2)

    def getCurrentDraw(self) -> wpimath.units.amperes:
        return self.driveSparkSim.getMotorCurrent() + self.steerSparkSim.getMotorCurrent()


class BatterySim2175():
    def __init__(self, *, nt: ntutil.Folder = ntutil.DummyFolder()):
        self.currentDraw: wpimath.units.amperes = 0
        self.voltageTopic = nt.getFloatTopic("Voltage")
        self.currentTopic = nt.getFloatTopic("Current")

    def iterate(self, current_draw: wpimath.units.amperes, dt: float):
        self.currentDraw = current_draw
        self.voltageTopic.set(self.outputVoltage())
        self.currentTopic.set(self.outputCurrent())

    def outputVoltage(self) -> wpimath.units.volts:
        return 0

    def outputCurrent(self) -> wpimath.units.amperes:
        return self.currentDraw


class NormalBatterySim(BatterySim2175):
    def __init__(
        self,
        volts: wpimath.units.volts = 12,
        *, nt: ntutil.Folder = ntutil.DummyFolder(),
    ):
        super().__init__(nt=nt)
        self.volts = volts

    def outputVoltage(self):
        return self.volts


class DrainingBatterySim(BatterySim2175):
    # Empirical good-enough voltage values for an FRC battery as it drains.
    MAX_OUTPUT_VOLTS: wpimath.units.volts = 12.9
    MIN_OUTPUT_VOLTS: wpimath.units.volts = 10.5

    # Less than the nominal amp-hours of 18, for Realism.
    FULL_BATTERY_AMP_HOURS: wpimath.units.ampere_hours = 12

    def __init__(self,
        *,
        usable_amp_hours: wpimath.units.ampere_hours | None = None,
        nt: ntutil.Folder = ntutil.DummyFolder(),
    ):
        super().__init__(nt=nt)
        self.charge: wpimath.units.ampere_hours = DrainingBatterySim.FULL_BATTERY_AMP_HOURS if usable_amp_hours is None else usable_amp_hours

    def iterate(self, current_draw: wpimath.units.amperes, dt: float):
        spentCharge: wpimath.units.ampere_hours = current_draw * (dt / 60 / 60)
        self.currentDraw = current_draw
        self.charge = max(0, self.charge - spentCharge)
        super().iterate(current_draw, dt)

    def nominalVoltage(self) -> wpimath.units.volts:
        return utils.remap(
            self.charge,
            (DrainingBatterySim.FULL_BATTERY_AMP_HOURS, 0),
            (DrainingBatterySim.MAX_OUTPUT_VOLTS, DrainingBatterySim.MIN_OUTPUT_VOLTS),
        )

    def outputVoltage(self) -> wpimath.units.volts:
        return BatterySim.calculate(self.nominalVoltage(), 0.020, currents=[self.currentDraw])


class SparkMaxSim2175(SparkMaxSim):
    """
    A wrapper around rev's SparkMaxSim that automatically performs torque
    calculations and logs sim-specific state to NetworkTables. Also overrides
    electrical-current calculations on iterate.
    """

    def __init__(
        self,
        sparkMax: SparkMax,
        motor: DCMotor,
        *, gearReduction: float = 1,
        nt: ntutil.Folder = ntutil.DummyFolder(),
    ) -> None:
        super().__init__(sparkMax, motor)
        self.realSpark = sparkMax
        self.encoder = self.getRelativeEncoderSim()
        self.gearboxedMotor = motor.withReduction(gearReduction)
        self.gearReduction = gearReduction

        self.lastCurrent: wpimath.units.amperes = 0
        self.lastTorque: wpimath.units.newton_meters = 0

        self.dutyCycleTopic = nt.getFloatTopic("DutyCycle")
        self.velocityTopic = nt.getFloatTopic("SimVelocity")
        self.gearboxVelocityRadiansTopic = nt.getFloatTopic("SimGearboxVelocityRadians")
        self.currentTopic = nt.getFloatTopic("SimCurrent")
        self.torqueTopic = nt.getFloatTopic("Torque")

    def iterate(self, velocity, vbus, dt):
        super().iterate(velocity, vbus, dt)

        # Warnings about SupportsFloat vs. float are annoying.
        velocity = float(velocity)
        vbus = float(vbus)
        dt = float(dt)

        # Compute output current & torque. In principle it seems like maybe we
        # should be able to just call getMotorCurrent(), but this doesn't even
        # seem to take coast mode into account, so overriding the current-
        # calculation logic seems prudent.
        outputDutyCycle = self.getAppliedOutput()
        isCoast = self.realSpark.configAccessor.getIdleMode() == rev.SparkBaseConfig.IdleMode.kCoast
        if outputDutyCycle == 0 and isCoast:
            outputCurrent = 0
        else:
            outputCurrent = self.gearboxedMotor.current(self.gearboxVelocityRadians(velocity), outputDutyCycle * vbus)
        outputTorque = self.gearboxedMotor.torque(outputCurrent)
        
        self.lastCurrent = outputCurrent
        self.lastTorque = outputTorque

        self.dutyCycleTopic.set(outputDutyCycle)
        self.velocityTopic.set(velocity)
        self.gearboxVelocityRadiansTopic.set(self.gearboxVelocityRadians(velocity))
        self.currentTopic.set(outputCurrent)
        self.torqueTopic.set(outputTorque)

    def gearboxVelocityRadians(self, velocityConverted: float) -> wpimath.units.radians_per_second:
        """
        Gets the velocity in radians per second of the SPARK MAX's motor,
        *after* gear reduction is applied. This is because
        DCMotor.withReduction() "bakes in" the gear reduction. Conceptually,
        you should think of the new "with reduction" motor as a new black-box
        device with its own free speed. And since this is the object we use in
        order to compute current and torque, the velocity that we pass it (in
        rad/sec) must therefore be the speed of the gearbox's output shaft.
        """
        motorRadPerSec = velocityConverted / self.encoder.getVelocityConversionFactor() * (2 * math.pi / 60)
        return motorRadPerSec / self.gearReduction

    def getMotorCurrent(self) -> wpimath.units.amperes:
        return self.lastCurrent

    def getMotorTorque(self) -> wpimath.units.newton_meters:
        return self.lastTorque
