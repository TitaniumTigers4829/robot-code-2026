// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterInterface {
  @AutoLog
  public static class ShooterInputs { // For values
    public double flywheelRPM = 0.0;
    public double flywheelMotorVoltage = 0.0;
    public double flywheelDutyCycle = 0.0;
    public double flywheelDesiredRPM = 0.0;
    public double flywheelStatorCurrent = 0.0;
    public double flywheelVelocity = 0.0;
    public double flywheelRPMError = 0.0;
    public double flywheelMotorTemp = 0.0;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default double getFlywheelRPM() {
    return 0.0;
  }

  public default void setFlywheelRPM(double targetRPM) {}

  public default void setVolts(double volts) {}

  public default void setPercentOutput(double distance) {}

  public default double getVolts() {
    return 0.0;
  }

  public default void setSpeed(double speed) {}

  public default void openLoop(double output) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}
}
