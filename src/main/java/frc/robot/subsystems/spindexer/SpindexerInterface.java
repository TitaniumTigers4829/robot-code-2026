// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SpindexerInterface {
  @AutoLog
  public static class SpindexerInputs { // For values

    public double spindexerMotorVoltage = 0.0;
    public double spindexerDutyCycle = 0.0;

    public double spindexerStatorCurrent = 0.0;
    public double spindexerVelocity = 0.0;

    public double spindexerMotorTemp = 0.0;
  }

  public default void updateInputs(SpindexerInputs inputs) {}

  public default double getSpindexerSpeed() {
    return 0.0;
  }

  public default void setSpindexerSpeed(double targetRPM) {}

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
