// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {

  /* -------------------------------------------------------------------------- */
  /*                               HARDWARE IDs                                 */
  /* -------------------------------------------------------------------------- */

  public static final int TURRET_MOTOR_ID = 22;
  public static final int TURRET_CANCODER_ID = 2;

  /* -------------------------------------------------------------------------- */
  /*                                GEAR RATIOS                                 */
  /* -------------------------------------------------------------------------- */

  // Total motor-to-turret gear ratio: 22.22222222222222222:1
  public static final double TOTAL_GEAR_RATIO = 22.22222222222;

  // The CANcoder sits on an intermediate shaft between the motor and turret output.
  //   CANcoder shaft → Turret output: 10:1
  //   Motor → CANcoder shaft: 22.22222222222222222 / 10 ≈ 2.2222:1
  // Used by PhysicalTurret for CRT seeding on boot.
  public static final double CANCODER_TO_TURRET = 10;

  /* -------------------------------------------------------------------------- */
  /*                            TURRET GEOMETRY                                 */
  /* -------------------------------------------------------------------------- */

  // Distance from center of robot to center of turret (turret is at front left)
  // TODO: Maybe change back tmr, i flipped this night before elon day 2
  public static final double Y_OFFSET = Units.inchesToMeters(7.75);
  public static final double X_OFFSET = Units.inchesToMeters(4.68);

  public static final Translation2d TURRET_OFFSET =
      new Translation2d(TurretConstants.X_OFFSET, TurretConstants.Y_OFFSET);

  /* -------------------------------------------------------------------------- */
  /*                            MOTION MAGIC LIMITS                             */
  /* -------------------------------------------------------------------------- */

  public static double MAX_VELOCITY_ROTATIONS_PER_SECOND = 10;
  public static double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 40;

  /* -------------------------------------------------------------------------- */
  /*                             MANUAL CONTROL                                 */
  /* -------------------------------------------------------------------------- */

  public static double CCW_MANUAL_SPEED = 0.1;
  public static double CW_MANUAL_SPEED = -0.1;

  /* -------------------------------------------------------------------------- */
  /*                                   PID / FF                                 */
  /* -------------------------------------------------------------------------- */

  // TODO: Tune these on actual hardware
  public static double TURRET_P = 9.4829;
  public static double TURRET_I = 0;
  public static double TURRET_D = 0;
  public static double TURRET_S = 0.22;
  public static double TURRET_V = 0.1;
  public static double TURRET_A = 0;

  /* -------------------------------------------------------------------------- */
  /*                              ANGLE TOLERANCES                              */
  /* -------------------------------------------------------------------------- */

  // All values in turret rotations (not motor rotations)
  public static double TURRET_ERROR_TOLERANCE = 0;
  public static double TURRET_DEADBAND = 0.001;

  // Soft limits: ±0.5 rotations = ±180°
  // NOTE: enforce these in TurretSubsystem.setTurretAngle(), NOT via Phoenix
  // soft limits — ContinuousWrap and Phoenix soft limits conflict with each other.
  // public static double MAX_ANGLE = 5.35;
  // public static double MIN_ANGLE = -3.291748;
  public static double MAX_ANGLE = 3.81;
  public static double MIN_ANGLE = -6.55;

  /* -------------------------------------------------------------------------- */
  /*                               CANCODER CONFIG                              */
  /* -------------------------------------------------------------------------- */

  public static final SensorDirectionValue ENCODER_REVERSED =
      SensorDirectionValue.Clockwise_Positive;

  // Magnet offset found via Phoenix Tuner X with turret physically at forward (0°).
  // To re-zero: point turret forward, read raw CANcoder absolute position in
  // Tuner X, and update this value.
  public static final double ANGLE_ZERO = -0.036376953125;

  /* -------------------------------------------------------------------------- */
  /*                            CURRENT LIMITS                                  */
  /* -------------------------------------------------------------------------- */

  public static final double STATOR_CURRENT_LIMIT = 100;
  public static final double SUPPLY_CURRENT_LIMIT = 100;
  public static final boolean STATOR_CURRENT_LIMIT_ENABLE = false;
  public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = false;

  // 5.630615
  // -3.291748

  // 9

  // chud
  // chuzz
}
