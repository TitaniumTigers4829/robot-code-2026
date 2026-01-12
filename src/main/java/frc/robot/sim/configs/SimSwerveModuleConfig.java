package frc.robot.sim.configs;

import edu.wpi.first.util.struct.StructSerializable;

public class SimSwerveModuleConfig implements StructSerializable {
  public SimMechanismConfig driveConfig;
  public SimMechanismConfig steerConfig;
  public double tireCoefficientOfFriction;
  public double wheelsRadiusMeters;

  public SimSwerveModuleConfig(
      SimMechanismConfig driveConfig,
      SimMechanismConfig steerConfig,
      double tireCoefficientOfFriction,
      double wheelsRadiusMeters) {
    this.driveConfig = driveConfig;
    this.steerConfig = steerConfig;
    this.tireCoefficientOfFriction = tireCoefficientOfFriction;
    this.wheelsRadiusMeters = wheelsRadiusMeters;
  }

  /** Stores the coefficient of friction of some common used wheels. */
  public enum WheelCof {
    // https://www.chiefdelphi.com/t/spectrum-3847-build-blog-2024/447471/217
    COLSONS(0.9),
    BLACK_NITRILE(1.542),
    VEX_GRIPLOCK_V2(1.916);

    public final double cof;

    WheelCof(double cof) {
      this.cof = cof;
    }
  }
}
