// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TurretInterface {
  @AutoLog
  public static class TurretInputs { // For values
    public double turretAngle = 0.0;
    public double turretAngularVelocity = 0.0;
    public double turretMotorAppliedVoltage = 0.0;
    public double turretDutyCycle = 0.0;
    public double turretDesiredAngle = 0.0;
    public double turretStatorCurrent = 0.0;
    public double turretAngleError = 0.0;
    public double turretMotorTemp = 0.0;
  }

  public default void updateInputs(TurretInputs inputs) {}

  /*in rotations and in relation to the hub */
  public default double getTurretAngle() {
    return 0.0;
  }

  public default void setTurretAngle(double desiredAngle) {}

  public default void setVolts(double volts) {}

  public default void setPercentOutput(double output) {}

  public default double getVolts() {
    return 0.0;
  }

  public default void setSpeed(double speed) {}

  public default void openLoop(double output) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}
}
