package frc.robot.sim.configs;

import edu.wpi.first.util.struct.StructSerializable;

public class SimGyroConfig implements StructSerializable {
  public double averageDriftingIn30SecsMotionlessDeg;
  public double velocityMeasurementStandardDeviationPercent;

  public SimGyroConfig(
      double averageDriftingIn30SecsMotionlessDeg,
      double velocityMeasurementStandardDeviationPercent) {
    this.averageDriftingIn30SecsMotionlessDeg = averageDriftingIn30SecsMotionlessDeg;
    this.velocityMeasurementStandardDeviationPercent = velocityMeasurementStandardDeviationPercent;
  }

  public SimGyroConfig() {}

  public SimGyroConfig withAverageDriftingIn30SecsMotionlessDeg(
      double averageDriftingIn30SecsMotionlessDeg) {
    this.averageDriftingIn30SecsMotionlessDeg = averageDriftingIn30SecsMotionlessDeg;
    return this;
  }

  public SimGyroConfig withVelocityMeasurementStandardDeviationPercent(
      double velocityMeasurementStandardDeviationPercent) {
    this.velocityMeasurementStandardDeviationPercent = velocityMeasurementStandardDeviationPercent;
    return this;
  }

  /**
   *
   *
   * <h2>Creates the Simulation for a <a href="https://store.ctr-electronics.com/pigeon-2/">CTRE
   * Pigeon 2 IMU</a>. </h2>
   *
   * @return a gyro simulation factory configured for the Pigeon 2 IMU
   */
  public static SimGyroConfig ofPigeon2() {
    /*
     * user manual of pigeon 2:
     * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
     */
    return new SimGyroConfig(0.5, 0.02);
  }

  /**
   *
   *
   * <h2>Creates the Simulation for a <a href="https://pdocs.kauailabs.com/navx-mxp/">navX2-MXP
   * IMU</a>.</h2>
   *
   * @return a gyro simulation factory configured for the navX2-MXP IMU
   */
  public static SimGyroConfig ofNavX2() {
    return new SimGyroConfig(.002, 0.004);
  }
}
