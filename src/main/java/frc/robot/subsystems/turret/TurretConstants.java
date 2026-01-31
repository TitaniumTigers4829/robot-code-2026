// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class TurretConstants {
    
  public static final int TURRET_MOTOR_ID = 0-9;
  public static final int TURRET_CANCODER_ID = 0-9;
  public static final double GEAR_RATIO = 0;

  public static final double X_DISTANCE = Units.inchesToMeters(-7.75);
  public static final double Y_DISTANCE = Units.inchesToMeters(4.68);
  public static final double Y_DISTANCE_FROM_FRONT = Units.inchesToMeters(7.25);

  public static double MAX_VELOCITY_ROTATIONS_PER_SECOND = 10;
  public static double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 4;
  
  //TODO: Tune
  public static double TURRET_P = 0;
  public static double TURRET_I = 0;
  public static double TURRET_D = 0;
  public static Constraints TURRET_CONSTRAINTS =
    new Constraints(MAX_VELOCITY_ROTATIONS_PER_SECOND, MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED);

  

  // in rotations
  public static double TURRET_ERROR_TOLERANCE = 0.01;
  public static double TURRET_DEADBAND = 0.001;

  public static final SensorDirectionValue ENCODER_REVERSED =
        SensorDirectionValue.Clockwise_Positive;
  //TODO: Find zero angle
  public static final double ANGLE_ZERO = 0;

  //TODO: Find limits, currently both are disabled
  public static final double STATOR_CURRENT_LIMIT = 100;
  public static final double SUPPLY_CURRENT_LIMIT = 100;

  public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;
  public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = false;
}
