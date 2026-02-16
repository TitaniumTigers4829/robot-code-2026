// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbInterface {
  @AutoLog
  public static class ClimbInputs { // For values
    public double climbMotorVoltage = 0.0;
    public double climbDutyCycle = 0.0;
    public double climbStatorCurrent = 0.0;
    public double climbVelocity = 0.0;
    public double climbMotorTemp = 0.0;
    public double climbHeight = 0.0;
  }

  public default void updateInputs(ClimbInputs inputs) {}

  public default void setVolts(double volts) {}

  public default void setPercentOutput(double distance) {}

  public default double getVolts() {
    return 0.0;
  }


  public default void openLoop(double output) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void levelOne() {}
}

