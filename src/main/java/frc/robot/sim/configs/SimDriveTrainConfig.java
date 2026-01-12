package frc.robot.sim.configs;

import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.sim.simMechanism.SimDriveTrain;

/**
 *
 *
 * <h1>Stores the configurations for a swerve drive simulation.</h1>
 *
 * <p>This class is used to hold all the parameters necessary for simulating a drive drivetrain,
 * allowing for realistic performance testing and evaluation.
 */
public class SimDriveTrainConfig<T extends SimDriveTrain, S extends SimDriveTrainConfig<T, S>>
    implements StructSerializable {
  public double robotMassKg, robotMoI, bumperLengthXMeters, bumperWidthYMeters;

  /**
   *
   *
   * <h2>Ordinary Constructor</h2>
   *
   * <p>Creates an instance of {@link SimDriveTrainConfig} with specified parameters.
   *
   * @param robotMassKg the mass of the robot in kilograms, including bumpers.
   * @param robotMoI the moment of inertia of the robot in kilograms per square meter.
   * @param bumperLengthXMeters the length of the bumper in meters (distance from front to back).
   * @param bumperWidthYMeters the width of the bumper in meters (distance from left to right).
   */
  protected SimDriveTrainConfig(
      double robotMassKg, double robotMoI, double bumperLengthXMeters, double bumperWidthYMeters) {
    this.robotMassKg = robotMassKg;
    this.robotMoI = robotMoI;
    this.bumperLengthXMeters = bumperLengthXMeters;
    this.bumperWidthYMeters = bumperWidthYMeters;
  }

  /**
   *
   *
   * <h2>Sets the robot mass.</h2>
   *
   * <p>Updates the mass of the robot in kilograms.
   *
   * @param robotMassKg the new mass of the robot in kilograms.
   * @return the current instance of {@link SimDriveTrainConfig} for method chaining.
   */
  @SuppressWarnings("unchecked")
  public S withRobotMass(double robotMassKg) {
    this.robotMassKg = robotMassKg;
    return (S) this;
  }

  /**
   *
   *
   * <h2>Sets the moment of inertia of the robot.</h2>
   *
   * <p>Updates the moment of inertia of the robot in kilograms per square meter.
   *
   * @param robotMoI the new moment of inertia of the robot in kilograms per square meter.
   * @return the current instance of {@link SimDriveTrainConfig} for method chaining.
   */
  @SuppressWarnings("unchecked")
  public S withRobotMoI(double robotMoI) {
    this.robotMoI = robotMoI;
    return (S) this;
  }

  /**
   *
   *
   * <h2>Sets the bumper size.</h2>
   *
   * <p>Updates the dimensions of the bumper.
   *
   * @param bumperLengthXMeters the length of the bumper in meters.
   * @param bumperWidthYMeters the width of the bumper in meters.
   * @return the current instance of {@link SimDriveTrainConfig} for method chaining.
   */
  @SuppressWarnings("unchecked")
  public S withBumperSize(double bumperLengthXMeters, double bumperWidthYMeters) {
    this.bumperLengthXMeters = bumperLengthXMeters;
    this.bumperWidthYMeters = bumperWidthYMeters;
    return (S) this;
  }

  /**
   *
   *
   * <h2>Calculates the density of the robot.</h2>
   *
   * <p>Returns the density of the robot based on its mass and bumper dimensions.
   *
   * @return the density in kilograms per square meter.
   */
  public double getDensity() {
    return robotMassKg / (bumperLengthXMeters * bumperWidthYMeters);
  }

  // @SuppressWarnings("rawtypes")
  // public static final Struct<SimDriveTrainConfig> struct =
  // ProceduralStructGenerator.genObjectNoUnpack(
  //     SimDriveTrainConfig.class
  // );
}
