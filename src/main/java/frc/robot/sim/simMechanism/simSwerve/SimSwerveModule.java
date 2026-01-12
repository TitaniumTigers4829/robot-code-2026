package frc.robot.sim.simMechanism.simSwerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.extras.logging.RuntimeLog;
import frc.robot.extras.math.forces.Velocity2d;
import frc.robot.extras.math.mathutils.MeasureMath;
import frc.robot.extras.math.mathutils.MeasureMath.XY;
import frc.robot.sim.SimRobot;
import frc.robot.sim.configs.SimSwerveConfig;
import frc.robot.sim.configs.SimSwerveModuleConfig;
import frc.robot.sim.simController.SimMotorController;
import frc.robot.sim.simField.SimArena.SimEnvTiming;
import frc.robot.sim.simMechanism.SimMechanism;
import frc.robot.sim.simMechanism.SimMechanism.MechanismState;
import frc.robot.sim.simMechanism.SimMechanism.MechanismVariables;
import java.util.function.Supplier;

public class SimSwerveModule {
  private final SimRobot<SimSwerve> robot;
  private final SimMechanism driveMech;
  private final SimMechanism steerMech;
  private final double wheelsCoefficientOfFriction;
  private final Force gravityForce;
  private final Distance wheelRadius;
  private final Translation2d translation;
  private final int moduleId;

  private final SimEnvTiming timing;

  SimSwerveModule(
      SimRobot<SimSwerve> robot,
      SimSwerveConfig config,
      int moduleId,
      Force gravityForce,
      Supplier<MomentOfInertia> rotorInertia,
      SimMotorController driveController,
      SimMotorController steerController) {
    this.robot = robot;
    final SimSwerveModuleConfig moduleConfig = config.swerveModuleConfig;
    timing = robot.timing();
    this.gravityForce = gravityForce;
    translation = config.moduleTranslations[moduleId];
    driveMech =
        new SimMechanism(
            "drive" + moduleId,
            moduleConfig.driveConfig.motor,
            driveController,
            moduleConfig.driveConfig.rotorInertia,
            moduleConfig.driveConfig.gearRatio,
            moduleConfig.driveConfig.friction,
            moduleConfig.driveConfig.dynamics,
            moduleConfig.driveConfig.limits,
            moduleConfig.driveConfig.noise,
            timing);
    steerMech =
        new SimMechanism(
            "drive" + moduleId,
            moduleConfig.steerConfig.motor,
            steerController,
            moduleConfig.steerConfig.rotorInertia,
            moduleConfig.steerConfig.gearRatio,
            moduleConfig.steerConfig.friction,
            moduleConfig.steerConfig.dynamics,
            moduleConfig.steerConfig.limits,
            moduleConfig.steerConfig.noise,
            timing);
    robot.addMechanism(driveMech);
    robot.addMechanism(steerMech);
    wheelsCoefficientOfFriction = config.swerveModuleConfig.tireCoefficientOfFriction;
    wheelRadius = Meters.of(config.swerveModuleConfig.wheelsRadiusMeters);
    this.moduleId = moduleId;

    RuntimeLog.debug("Created a swerve module simulation");
  }

  public record ModuleMotorPair<T>(T drive, T steer) {}

  public ModuleMotorPair<MechanismVariables> inputs() {
    return new ModuleMotorPair<>(driveMech.variables(), steerMech.variables());
  }

  public ModuleMotorPair<MechanismState> outputs() {
    return new ModuleMotorPair<>(driveMech.state(), steerMech.state());
  }

  public SwerveModuleState state() {
    return new SwerveModuleState(
        driveMech.state().velocity().in(RadiansPerSecond) * wheelRadius.in(Meters),
        new Rotation2d(steerMech.state().position()));
  }

  protected void teardown() {
    robot.removeMechanism(driveMech);
    robot.removeMechanism(steerMech);
  }

  protected Translation2d translation() {
    return translation;
  }

  protected int id() {
    return moduleId;
  }

  protected XY<Force> force(final Rotation2d robotHeading) {
    final Rotation2d steerMechAngle =
        new Rotation2d(steerMech.state().position()).plus(robotHeading);
    final Force gripForce = gravityForce.times(wheelsCoefficientOfFriction);
    final Force driveMechAppliedForce = driveMech.variables().torque().div(wheelRadius);

    final boolean isSkidding = MeasureMath.abs(driveMechAppliedForce).gt(gripForce);
    final Force propellingForce;
    if (isSkidding) {
      propellingForce = gripForce.times(MeasureMath.signum(driveMechAppliedForce));
    } else {
      propellingForce = driveMechAppliedForce;
    }

    return new XY<Force>(
        propellingForce.times(steerMechAngle.getCos()),
        propellingForce.times(steerMechAngle.getSin()));
  }

  protected XY<Force> friction(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading) {
    final Force gripForce = gravityForce.times(wheelsCoefficientOfFriction);

    final Distance drivebaseRadius = Meters.of(translation.getNorm());
    final LinearVelocity tangentialVelocity =
        MetersPerSecond.of(chassisSpeeds.omegaRadiansPerSecond * drivebaseRadius.in(Meters));
    final Rotation2d tangentialAngle =
        translation.getAngle().rotateBy(robotHeading).rotateBy(Rotation2d.kCCW_90deg);
    final Velocity2d tangentialVelocityVector =
        new Velocity2d(
            tangentialVelocity.times(tangentialAngle.getCos()),
            tangentialVelocity.times(tangentialAngle.getSin()));
    final Velocity2d moduleWorldVelocity =
        new Velocity2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    final Velocity2d moduleDriveVelocity =
        new Velocity2d(
            driveMech.state().velocity().in(RadiansPerSecond) * wheelRadius.in(Meters),
            new Rotation2d(steerMech.state().position()).plus(robotHeading));
    final Velocity2d unwantedVelocity =
        moduleWorldVelocity.plus(tangentialVelocityVector).minus(moduleDriveVelocity);

    return new XY<Force>(
        gripForce.times(-Math.signum(unwantedVelocity.getVX())),
        gripForce.times(-Math.signum(unwantedVelocity.getVY())));
  }
}
