package frc.robot.sim.configs;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.extras.util.ProceduralStructGenerator.FixedSizeArray;
import frc.robot.sim.simMechanism.simSwerve.SimSwerve;
import java.util.Arrays;
import java.util.OptionalDouble;

/**
 *
 *
 * <h1>Stores the configurations for a swerve drive simulation.</h1>
 *
 * <p>This class is used to hold all the parameters necessary for simulating a swerve drivetrain,
 * allowing for realistic performance testing and evaluation.
 */
public class SimSwerveConfig extends SimDriveTrainConfig<SimSwerve, SimSwerveConfig> {
  public SimSwerveModuleConfig swerveModuleConfig;
  public SimGyroConfig gyroConfig;

  @FixedSizeArray(size = 4)
  public Translation2d[] moduleTranslations;

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
   * @param trackLengthXMeters the distance between the front and rear wheels, in meters.
   * @param trackWidthYMeters the distance between the left and right wheels, in meters.
   * @param swerveModuleSimulationFactory the factory that creates appropriate swerve module
   *     simulation for the drivetrain.
   * @param gyroSimulationFactory the factory that creates appropriate gyro simulation for the
   *     drivetrain.
   */
  public SimSwerveConfig(
      double robotMassKg,
      double robotMoI,
      double bumperLengthXMeters,
      double bumperWidthYMeters,
      Translation2d[] moduleTranslations,
      SimSwerveModuleConfig swerveModuleConfig,
      SimGyroConfig gyroConfig) {
    super(robotMassKg, robotMoI, bumperLengthXMeters, bumperWidthYMeters);

    this.swerveModuleConfig = swerveModuleConfig;
    this.gyroConfig = gyroConfig;
    this.moduleTranslations = moduleTranslations;
  }

  /**
   *
   *
   * <h2>Sets custom module translations.</h2>
   *
   * <p>Updates the translations of the swerve modules with user-defined values.
   *
   * <p>For ordinary rectangular modules configuration, use {@link
   * #withTrackLengthTrackWidth(double, double)} instead.
   *
   * @param moduleTranslations the custom translations for the swerve modules.
   * @return the current instance of {@link SimSwerveConfig} for method chaining.
   */
  public SimSwerveConfig withModuleTranslations(Translation2d[] moduleTranslations) {
    this.moduleTranslations = moduleTranslations;
    return this;
  }

  /**
   *
   *
   * <h2>Sets the swerve module config.</h2>
   *
   * @param swerveModuleConfig the new config for swerve module simulations.
   * @return the current instance of {@link SimSwerveConfig} for method chaining.
   */
  public SimSwerveConfig withSwerveModule(SimSwerveModuleConfig swerveModuleConfig) {
    this.swerveModuleConfig = swerveModuleConfig;
    return this;
  }

  /**
   *
   *
   * <h2>Sets the gyro config.</h2>
   *
   * @param gyroConfig the new config for gyro simulation.
   * @return the current instance of {@link SimSwerveConfig} for method chaining.
   */
  public SimSwerveConfig withGyro(SimGyroConfig gyroConfig) {
    this.gyroConfig = gyroConfig;
    return this;
  }

  /**
   *
   *
   * <h2>Calculates the track length in the X direction.</h2>
   *
   * <p>Returns the total distance between the frontmost and rearmost module translations in the X
   * direction.
   *
   * @return the track length in meters.
   * @throws IllegalStateException if the module translations are empty.
   */
  public double getTrackLengthX() {
    final OptionalDouble maxModuleX =
        Arrays.stream(moduleTranslations).mapToDouble(Translation2d::getX).max();
    final OptionalDouble minModuleX =
        Arrays.stream(moduleTranslations).mapToDouble(Translation2d::getX).min();
    if (maxModuleX.isEmpty() || minModuleX.isEmpty())
      throw new IllegalStateException("Modules translations are empty");
    return maxModuleX.getAsDouble() - minModuleX.getAsDouble();
  }

  /**
   *
   *
   * <h2>Calculates the track width in the Y direction.</h2>
   *
   * <p>Returns the total distance between the leftmost and rightmost module translations in the Y
   * direction.
   *
   * @return the track width in meters.
   * @throws IllegalStateException if the module translations are empty.
   */
  public double getTrackWidthY() {
    final OptionalDouble maxModuleY =
        Arrays.stream(moduleTranslations).mapToDouble(Translation2d::getY).max();
    final OptionalDouble minModuleY =
        Arrays.stream(moduleTranslations).mapToDouble(Translation2d::getY).min();
    if (maxModuleY.isEmpty() || minModuleY.isEmpty())
      throw new IllegalStateException("Modules translations are empty");
    return maxModuleY.getAsDouble() - minModuleY.getAsDouble();
  }

  // public static final Struct<SimSwerveConfig> struct =
  // ProceduralStructGenerator.genObjectNoUnpack(SimSwerveConfig.class);
}
