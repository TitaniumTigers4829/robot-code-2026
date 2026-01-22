// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {
  public static int FLYWHEEL_MOTOR_ID = 0 - 9;

  public static double SHOOTER_HEIGHT_FROM_GROUND = 0;

  public static double FLYWHEEL_S = 0;
  public static double FLYWHEEL_V = 0;
  public static double FLYWHEEL_A = 0;
  public static double FLYWHEEL_P = 0;
  public static double FLYWHEEL_I = 0;
  public static double FLYWHEEL_D = 0;

  // 5 rpm
  public static double FLYWHEEL_ERROR_TOLERANCE = 5;

  // Lookup table for rpms needed for certain distances
  // TODO: Add actual tested values
  public static double[][] DISTANCE_TO_FLYWHEEL_RPM = {
    // Distance from hub in meters, needed rpm of flywheel
    {0, 0},
    {0, 0},
    {0, 0},
  };
}
