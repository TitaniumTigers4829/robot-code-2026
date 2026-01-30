// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class TurretConstants {

public static final int TURRET_MOTOR_ID = 0-9;
  public static final double GEAR_RATIO = 0;

  public static final double X_DISTANCE = Units.inchesToMeters(-7.75);
  public static final double Y_DISTANCE = Units.inchesToMeters(4.68);
  public static final double Y_DISTANCE_FROM_FRONT = Units.inchesToMeters(7.25);

  public static double TURRET_S = 0;
  public static double TURRET_V = 0;
  public static double TURRET_A = 0;
  public static double TURRET_P = 0;
  public static double TURRET_I = 0;
  public static double TURRET_D = 0;

  // in rotations
  public static double TURRET_ERROR_TOLERANCE = 0.01;
}
