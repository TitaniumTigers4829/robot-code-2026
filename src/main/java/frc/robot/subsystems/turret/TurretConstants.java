// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class TurretConstants {

  public static final int TURRET_MOTOR_ID = 22;
  public static final int TURRET_CANCODER_ID = 0 - 9;
  public static final double GEAR_RATIO = 40;

  // Distance from center of robot to center of turret in x and y directions, turret is at front
  // left
  public static final double X_OFFSET = Units.inchesToMeters(-7.75);
  public static final double Y_OFFSET = Units.inchesToMeters(4.68);
  //   public static final double Y_DISTANCE_FROM_FRONT = Units.inchesToMeters(7.25);

  // Creates a translation for the turret offset from the center of the robot
  public static final Translation2d TURRET_OFFSET =
      new Translation2d(TurretConstants.X_OFFSET, TurretConstants.Y_OFFSET);

  public static double MAX_VELOCITY_ROTATIONS_PER_SECOND = 10;
  public static double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 4;

  // For manual control
  public static double CCW_MANUAL_SPEED = 0.5;
  public static double CW_MANUAL_SPEED = -0.5;

  // TODO: Tune
  public static double TURRET_P = 1;
  public static double TURRET_I = 0;
  public static double TURRET_D = 0;
  public static double TURRET_S = 0;
  public static double TURRET_V = 0;
  public static double TURRET_A = 0;

  // in rotations
  public static double TURRET_ERROR_TOLERANCE = 0.01;
  public static double TURRET_DEADBAND = 0.001;
  public static double MAX_ANGLE = 1;
  public static double MIN_ANGLE = 0;

  public static final SensorDirectionValue ENCODER_REVERSED =
      SensorDirectionValue.Clockwise_Positive;

  // TODO: Find zero angle from phoenix tuner, should by facing front of robot
  public static final double ANGLE_ZERO = 0;

  // TODO: Find limits, currently both are disabled
  public static final double STATOR_CURRENT_LIMIT = 100;
  public static final double SUPPLY_CURRENT_LIMIT = 100;

  public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;
  public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = false;
}
