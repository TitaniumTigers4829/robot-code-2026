// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {

  public static int LEADER_FLYWHEEL_MOTOR_ID = 27;
  public static int FOLLOWER_FLYWHEEL_MOTOR_ID = 0;
  public static int KICKER_MOTOR_ID = 57;
  public static int SPINDEXER_MOTOR_ID = 61;

  public static double SHOOTER_HEIGHT_FROM_GROUND = 0;
  public static double GEAR_RATIO = 24 / 36;

  public static double FLYWHEEL_S = 0;
  public static double FLYWHEEL_V = 0;
  public static double FLYWHEEL_A = 0;
  public static double FLYWHEEL_P = 0;
  public static double FLYWHEEL_I = 0;
  public static double FLYWHEEL_D = 0;

  // 5 rpm
  public static double FLYWHEEL_ERROR_TOLERANCE = 5;

  public static double PASS_SHOOTER_SPEED = -1;
  public static double KICKER_PERCENT_OUTPUT = 0.95;
  public static double SPINDEXER_PERCENT_OUTPUT = 0.9; // 0.675

  // Lookup table for rpms needed for certain distances
  public static double[][] DISTANCE_TO_FLYWHEEL_RPM = {
    // Distance from hub in meters, needed rps of flywheel
    {0.5, 100},
    {1, 105},
    {1.5, 112},
    {2, 122},
    {2.5, 133.33333}
  };
}
