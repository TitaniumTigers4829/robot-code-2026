// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {

  public static int LEADER_FLYWHEEL_MOTOR_ID = 27;
  public static int FOLLOWER_FLYWHEEL_MOTOR_ID = 0;
  public static int KICKER_AND_ROLLER_MOTOR_ID = 57;
  public static int FRONT_ROLLER_MOTOR_ID = 00;

  public static double SHOOTER_HEIGHT_FROM_GROUND = 0;
  public static double GEAR_RATIO = 0.95;

  public static double FLYWHEEL_S = 0.2;
  public static double FLYWHEEL_V = 0.111;
  public static double FLYWHEEL_A = 0;
  public static double FLYWHEEL_P = 10000; // 0.1
  public static double FLYWHEEL_I = 0;
  public static double FLYWHEEL_D = 0;

  // 5 rpm
  public static double FLYWHEEL_ERROR_TOLERANCE = 2;

  public static double PASS_SHOOTER_SPEED = -1;
  public static double KICKER_PERCENT_OUTPUT = 0.5;
  public static double SPINDEXER_INTAKE_SPEED = 0.2; // 0.675
  public static double SPINDEXER_SHOOT_SPEED = 0.7; // 0.675

  public static double FLYWHEEL_STATOR_CURRENT_LIMIT = 80.0;
  public static double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 20.0;

  // Lookup table for rpms needed for certain distances
  public static double[][] DISTANCE_TO_FLYWHEEL_RPM = {
    // Distance from hub in meters, needed rps of flywheel
    {1.5, 48.29},
    {2.0, 52},
    {2.5, 57},
    {3.0, 57.4829},
    {3.5, 63.4829},
    {4.0, 64.4829},
    {4.5, 70}

    // 1.5, 0.3, 42
    // TODO: scuff
    // 2.5, 0.3, 65
    // 3.5, 0.4, 75
    // 3: 53:0.6
    // 4.1: 60:0.65
    // 188" (5.3m): 75:0.75
  };
}
