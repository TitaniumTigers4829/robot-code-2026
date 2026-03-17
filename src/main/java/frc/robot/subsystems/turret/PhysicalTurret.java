// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class PhysicalTurret implements TurretInterface {

  /* -------------------------------------------------------------------------- */
  /*                                  HARDWARE                                  */
  /* -------------------------------------------------------------------------- */

  private final TalonFX turretMotor =
      new TalonFX(TurretConstants.TURRET_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);

  private final CANcoder turretEncoder = new CANcoder(TurretConstants.TURRET_CANCODER_ID);

  // TODO(second-cancoder): Uncomment when second CANcoder is physically installed.
  // Mount this encoder on a separate gear with a different ratio to the turret ring
  // so that its gear ratio is co-prime with the first encoder's gear ratio.
  // See Team SCREAM 4522's write-up for gear tooth selection guidance.
  //
  // private final CANcoder turretEncoder2 =
  //     new CANcoder(TurretConstants.TURRET_CANCODER_2_ID);

  /* -------------------------------------------------------------------------- */
  /*                                CONFIG OBJECTS                              */
  /* -------------------------------------------------------------------------- */

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  // TODO(second-cancoder): Uncomment when second CANcoder is installed.
  // private final CANcoderConfiguration encoderConfig2 = new CANcoderConfiguration();

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0.0);

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);

  /* -------------------------------------------------------------------------- */
  /*                                GEAR RATIOS                                 */
  /* -------------------------------------------------------------------------- */

  // Total motor-to-turret gear ratio: 26.66667:1
  // i.e. the motor spins 26.666667 times for every 1 full turret rotation.
  private static final double TOTAL_RATIO = TurretConstants.TOTAL_GEAR_RATIO;

  // The CANcoder is NOT on the turret output. It sits on an intermediate shaft:
  //
  //   Motor ──(2.667:1)──► CANcoder shaft ──(16.666:1)──► Turret output
  //
  // MOTOR_TO_CANCODER = 44.444 / 16.666 ≈ 2.667
  // This means the CANcoder shaft spins 2.667 times per 1 turret rotation,
  // and the motor spins 2.667 times per 1 CANcoder shaft rotation.
  private static final double MOTOR_TO_CANCODER = TOTAL_RATIO / TurretConstants.CANCODER_TO_TURRET;

  // TODO(second-cancoder): Add second encoder gear ratio constant in TurretConstants.
  // The ratio must be co-prime with CANCODER_TO_TURRET (currently 16.666).
  // Example: if you use a 17T gear on the encoder and a 77T turret ring,
  // ratio = 77.0 / 17.0 ≈ 4.529. Check that gcd(numerator1, numerator2) = 1.
  //
  // private static final double MOTOR_TO_CANCODER_2 =
  //     TOTAL_RATIO / TurretConstants.CANCODER_2_TO_TURRET;

  /* -------------------------------------------------------------------------- */
  /*                                STATUS SIGNALS                              */
  /* -------------------------------------------------------------------------- */

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Angle> cancoderPosition;

  // TODO(second-cancoder): Uncomment when second CANcoder is installed.
  // private final StatusSignal<Angle> cancoderPosition2;

  /* -------------------------------------------------------------------------- */
  /*                                 CONSTRUCTOR                                */
  /* -------------------------------------------------------------------------- */

  public PhysicalTurret() {

    configureEncoder();
    configureMotor();

    motorPosition = turretMotor.getPosition();
    motorVelocity = turretMotor.getVelocity();
    motorVoltage = turretMotor.getMotorVoltage();
    motorCurrent = turretMotor.getStatorCurrent();
    motorTemp = turretMotor.getDeviceTemp();
    cancoderPosition = turretEncoder.getAbsolutePosition();

    // TODO(second-cancoder): Uncomment when second CANcoder is installed.
    // cancoderPosition2 = turretEncoder2.getAbsolutePosition();

    seedFalconFromAbsolute();

    motorPosition.setUpdateFrequency(250);
    motorVelocity.setUpdateFrequency(250);

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage, motorCurrent, motorTemp);

    // TODO(second-cancoder): Add turretEncoder2 to optimizeBusUtilizationForAll.
    ParentDevice.optimizeBusUtilizationForAll(turretMotor, turretEncoder);
  }

  /* -------------------------------------------------------------------------- */
  /*                               CONFIGURATION                                */
  /* -------------------------------------------------------------------------- */

  private void configureEncoder() {
    encoderConfig.MagnetSensor.MagnetOffset = TurretConstants.ANGLE_ZERO;
    encoderConfig.MagnetSensor.SensorDirection = TurretConstants.ENCODER_REVERSED;
    // encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 3;
    turretEncoder.getConfigurator().apply(encoderConfig);
  }

  // TODO(second-cancoder): Add configureEncoder2() when second CANcoder is installed.
  // Set ANGLE_ZERO_2 and ENCODER_2_REVERSED in TurretConstants.
  // The zero offset should be found the same way as the first — Phoenix Tuner X
  // with the turret physically at the hard stop / known reference position.
  //
  // private void configureEncoder2() {
  //   encoderConfig2.MagnetSensor.MagnetOffset = TurretConstants.ANGLE_ZERO_2;
  //   encoderConfig2.MagnetSensor.SensorDirection = TurretConstants.ENCODER_2_REVERSED;
  //   turretEncoder2.getConfigurator().apply(encoderConfig2);
  // }

  private void configureMotor() {

    // SensorToMechanismRatio = TOTAL_RATIO tells Phoenix 6 to automatically
    // divide all motor position/velocity signals by TOTAL_RATIO, so that
    // getPosition() returns turret output rotations, not raw motor rotations.
    // Do NOT divide by TOTAL_RATIO again anywhere when reading these signals.
    motorConfig.Feedback.SensorToMechanismRatio = TOTAL_RATIO;
    motorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // motorConfig.Feedback.RotorToSensorRatio = TurretConstants.CANCODER_TO_TURRET;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // ContinuousWrap lets the motor always take the shortest path to target.
    // NOTE: do not use Phoenix soft limits alongside ContinuousWrap — they conflict.
    // Enforce turret angle limits in TurretSubsystem.setTurretAngle() instead.
    // motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    motorConfig.Slot0.kP = TurretConstants.TURRET_P;
    motorConfig.Slot0.kI = TurretConstants.TURRET_I;
    motorConfig.Slot0.kD = TurretConstants.TURRET_D;
    motorConfig.Slot0.kS = TurretConstants.TURRET_S;
    motorConfig.Slot0.kV = TurretConstants.TURRET_V;
    motorConfig.Slot0.kA = TurretConstants.TURRET_A;

    motorConfig.MotionMagic.MotionMagicAcceleration =
        TurretConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        TurretConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;

    turretMotor.getConfigurator().apply(motorConfig);
  }

  /* -------------------------------------------------------------------------- */
  /*                         STARTUP ABSOLUTE ALIGNMENT                         */
  /* -------------------------------------------------------------------------- */

  /**
   * Seeds the Falcon's internal encoder with the true absolute turret position on boot, using a
   * single-encoder variation of the Chinese Remainder Theorem (CRT).
   *
   * <p>======================================================================== THE PROBLEM
   * ========================================================================
   *
   * <p>The CANcoder is a single-turn absolute encoder mounted on an intermediate shaft — NOT
   * directly on the turret output. It always reports a value between 0.0 and 1.0 representing its
   * fractional position within ONE shaft rotation, but it has no memory of how many full rotations
   * it has completed.
   *
   * <p>Because the intermediate shaft spins 2.667 times per turret rotation, a CANcoder reading of
   * 0.3 is consistent with the turret being at ANY of these:
   *
   * <p>0.3 / 2.667 = 0.112 turret rotations (~40°) 1.3 / 2.667 = 0.487 turret rotations (~175°) 2.3
   * / 2.667 = 0.862 turret rotations (~310°) ... and so on.
   *
   * <p>The CANcoder alone cannot tell us which of these is the real position. We need a second
   * piece of information to resolve the ambiguity.
   *
   * <p>======================================================================== THE SOLUTION
   * (current — single CANcoder + stale Falcon encoder)
   * ========================================================================
   *
   * <p>The Falcon's internal encoder value persists across power cycles — it is stored inside the
   * motor controller and survives a reboot. It may be slightly stale if the turret was physically
   * moved while the robot was off, but it is close enough to reality to tell us which of the ~2.667
   * possible positions the CANcoder reading corresponds to.
   *
   * <p>We convert the stale Falcon reading into estimated intermediate shaft rotations, then use
   * CRT to snap it to the nearest valid answer given what the CANcoder is actually reading right
   * now.
   *
   * <p>======================================================================== THE MATH — step by
   * step ========================================================================
   *
   * <p>Step 1 — Read the CANcoder fractional position (0.0–1.0). This value is always accurate
   * regardless of power history. Call it: cancoderRot
   *
   * <p>Step 2 — Convert the stale Falcon reading to intermediate shaft rotations. motorPosition
   * (turret rotations, stale) × TOTAL_RATIO = raw motor rotations raw motor rotations /
   * MOTOR_TO_CANCODER = estimatedCancoderRot This is approximate — possibly off by a fraction if
   * turret moved while off.
   *
   * <p>Step 3 — CRT: find k, the number of complete intermediate shaft rotations. k =
   * Math.round(estimatedCancoderRot - cancoderRot) trueCancoderRot = k + cancoderRot
   *
   * <p>Why this works: (estimatedCancoderRot - cancoderRot) is approximately equal to the integer
   * number of full shaft rotations. Math.round snaps it to the nearest whole number, giving us k.
   * We then add back the accurate fractional reading from the CANcoder to get the true total shaft
   * rotations.
   *
   * <p>Step 4 — Convert trueCancoderRot back to turret rotations and seed. trueMotorRot =
   * trueCancoderRot × MOTOR_TO_CANCODER finalTurretRot = trueMotorRot / TOTAL_RATIO
   * turretMotor.setPosition(finalTurretRot)
   *
   * <p>Worked example: cancoderRot = 0.3 ← CANcoder reads 0.3 shaft rotations right now turretRot
   * (stale) = 0.45 ← Falcon thinks turret is at 0.45 rotations motorRot = 0.45 × 44.444 = 20.0 raw
   * motor rotations estimatedCancoderRot = 20.0 / 2.667 = 7.5 intermediate shaft rotations k =
   * round(7.5 - 0.3) = round(7.2) = 7 trueCancoderRot = 7 + 0.3 = 7.3 intermediate shaft rotations
   * trueMotorRot = 7.3 × 2.667 = 19.47 raw motor rotations finalTurretRot = 19.47 / 44.444 = 0.438
   * turret rotations ✓
   *
   * <p>======================================================================== THE LIMIT — when
   * does this break? ========================================================================
   *
   * <p>Math.round picks the WRONG k if the Falcon's estimate is off by more than ±0.5 intermediate
   * shaft rotations from reality. This would happen if the turret was physically moved more than a
   * certain amount while powered off.
   *
   * <p>Safe window calculation: ±0.5 intermediate shaft rotations = ±0.5 / 2.667 turret rotations =
   * ±0.1875 turret rotations = ±0.1875 × 360° = ±67.5°
   *
   * <p>So the turret can be moved up to ±67.5° from forward while the robot is off and seeding will
   * still resolve correctly. This is a very wide window — eyeballing "roughly forward" before
   * powering on is more than sufficient.
   *
   * <p>If seeding IS wrong, it will be wrong by exactly one intermediate shaft rotation = 1/2.667
   * turret rotations = 135°. That's a big jump and will be immediately obvious during testing. The
   * sanity check below catches the case where the bad seed lands outside MIN/MAX. A bad seed that
   * happens to land INSIDE valid range cannot be detected without additional hardware.
   *
   * <p>======================================================================== PRE-MATCH PROCEDURE
   * ========================================================================
   *
   * <p>1. Before powering on the robot, physically rotate the turret so it is pointing roughly
   * toward the front of the robot. Eyeballing is fine — you have a ±67.5° window, which is very
   * generous.
   *
   * <p>2. Power on the robot. seedFalconFromAbsolute() runs automatically.
   *
   * <p>3. Check Driver Station for the "Turret seeding out of range!" warning. If it appears, the
   * turret was too far from forward. Manually drive the turret to forward using open loop and press
   * the rezero button.
   *
   * <p>4. If no warning appears, seeding succeeded. Verify by checking turret/seed/finalTurretRot
   * in AdvantageScope — it should be close to 0.0 if the turret was pointing forward.
   *
   * <p>========================================================================
   * TODO(second-cancoder): UPGRADE PATH TO TRUE TWO-ENCODER CRT
   * ========================================================================
   *
   * <p>When a second CANcoder is physically installed, this method should be REPLACED with a true
   * two-encoder CRT implementation. This eliminates the dependency on the stale Falcon encoder
   * entirely and makes seeding fully power-independent — the turret can be anywhere when you boot
   * and it will always resolve correctly.
   *
   * <p>HARDWARE REQUIREMENTS for two-encoder CRT: - Two absolute single-turn encoders - Each
   * encoder driven by a small gear off the turret ring - The tooth counts of the two encoder gears
   * must be CO-PRIME (their greatest common divisor must equal 1) - Easy way to pick co-prime
   * teeth: choose one prime number (e.g. 17), and pick any other number ≤ 18 that isn't a multiple
   * of 17 (e.g. 16) - The product (e1_teeth × e2_teeth) divided by turret ring teeth must be ≥ the
   * number of turret rotations you need to track
   *
   * <p>SOFTWARE — how the two-encoder algorithm works:
   *
   * <p>For each encoder, generate a list of ALL possible turret positions consistent with that
   * encoder's current reading:
   *
   * <p>for (int n = 0; n &lt; otherEncoder_teeth; n++) { double candidate = (n + encoderReading) /
   * gearRatio; add candidate to list; }
   *
   * <p>Do this for both encoders. The two lists will share exactly ONE value (within floating point
   * tolerance). That shared value is the true turret position. Example (from Team SCREAM 4522,
   * tooth counts 35T and 36T):
   *
   * <p>Encoder 1 reads 75° → possible positions: [0.097, 0.565, 1.032, 1.5, ...] Encoder 2 reads
   * 108° → possible positions: [0.136, 0.591, 1.045, 1.5, ...] Shared value: 1.5 rotations → turret
   * is at 1.5 × 360° = 540° from zero ✓
   *
   * <p>MATCHING — to find the shared value, iterate both lists and compare:
   *
   * <p>double TOLERANCE = 0.5 / turretRingTeeth; // half a tooth of tolerance for (double a :
   * list1) { for (double b : list2) { if (Math.abs(a - b) &lt; TOLERANCE) { // found it — seed
   * turretMotor with this value } } }
   *
   * <p>IMPORTANT NOTES for the upgrade: - Zero both encoders at the same known physical reference
   * position (e.g. turret at its hard stop) using Phoenix Tuner X - Add CANCODER_2_TO_TURRET and
   * ANGLE_ZERO_2 to TurretConstants - The stale Falcon encoder and MOTOR_TO_CANCODER are no longer
   * needed once this is implemented — you can remove them - See Team SCREAM 4522's 2024 write-up
   * for full worked examples - See all TODO(second-cancoder) comments throughout this file
   */
  private void seedFalconFromAbsolute() {

    cancoderPosition.refresh();
    motorPosition.refresh();

    // STEP 1: Read the CANcoder.
    // Returns the fractional position of the intermediate shaft (0.0–1.0).
    // This is always accurate — it's absolute and power-independent.
    double cancoderRot = cancoderPosition.getValueAsDouble();

    // STEP 2: Estimate the intermediate shaft position from the stale Falcon encoder.
    // motorPosition is already in turret rotations (Phoenix divided by TOTAL_RATIO).
    // We multiply back up to raw motor rotations, then divide by MOTOR_TO_CANCODER
    // to get how many full+fractional rotations the intermediate shaft has done.
    // This is stale but good enough to identify the correct integer revolution.
    double turretRot = motorPosition.getValueAsDouble(); // turret rotations (stale)
    double motorRot = turretRot * TOTAL_RATIO; // raw motor rotations (stale)
    double estimatedCancoderRot =
        motorRot / MOTOR_TO_CANCODER; // intermediate shaft rotations (stale)

    // STEP 3: CRT — resolve which revolution of the intermediate shaft we are on.
    // Subtracting cancoderRot from the estimate isolates the integer part (full rotations).
    // Math.round snaps it to the nearest whole number — this is valid as long as the
    // turret hasn't moved more than ±67.5° while powered off (see method javadoc).
    double k = Math.round(estimatedCancoderRot - cancoderRot);

    // Recombine: the true intermediate shaft position is k full rotations
    // plus the fractional position the CANcoder is currently reading.
    double trueCancoderRot = k + cancoderRot;

    // STEP 4: Convert back to turret rotations and seed the Falcon.
    // trueCancoderRot (intermediate shaft) → raw motor rotations → turret rotations.
    double trueMotorRot = trueCancoderRot * MOTOR_TO_CANCODER;
    double finalTurretRot = trueMotorRot / TOTAL_RATIO;

    // Sanity check: if the resolved position is outside physical limits, something
    // went wrong (turret moved too far while off, or first-ever boot with no history).
    // Fall back to 0.0 and fire a Driver Station warning so the drive team knows.
    // NOTE: a bad seed that happens to land within MIN/MAX will NOT be caught here —
    // that requires additional hardware such as a second CANcoder or hall effect sensor.
    boolean seedingValid =
        finalTurretRot >= TurretConstants.MIN_ANGLE && finalTurretRot <= TurretConstants.MAX_ANGLE;

    if (seedingValid) {
      turretMotor.setPosition(finalTurretRot);
    } else {
      turretMotor.setPosition(0.0);
      DriverStation.reportWarning(
          "Turret seeding out of range! Position defaulted to 0. "
              + "Point turret forward and press rezero before enabling.",
          false);
    }

    Logger.recordOutput("turret/seed/cancoderRot", cancoderRot);
    Logger.recordOutput("turret/seed/estimatedCancoderRot", estimatedCancoderRot);
    Logger.recordOutput("turret/seed/k", k);
    Logger.recordOutput("turret/seed/trueCancoderRot", trueCancoderRot);
    Logger.recordOutput("turret/seed/finalTurretRot", finalTurretRot);
    Logger.recordOutput("turret/seed/seedingValid", seedingValid);
  }

  /* -------------------------------------------------------------------------- */
  /*                                 IO UPDATE                                  */
  /* -------------------------------------------------------------------------- */

  @Override
  public void updateInputs(TurretInputs inputs) {

    BaseStatusSignal.refreshAll(
        motorPosition, motorVelocity, motorVoltage, motorCurrent, motorTemp);

    // Phoenix 6 already divides by SensorToMechanismRatio (TOTAL_RATIO) internally.
    // These values are already in turret output rotations — do NOT divide again.
    inputs.turretAngle = motorPosition.getValueAsDouble();
    inputs.turretAngularVelocity = motorVelocity.getValueAsDouble();

    inputs.turretMotorAppliedVoltage = motorVoltage.getValueAsDouble();
    inputs.turretStatorCurrent = motorCurrent.getValueAsDouble();
    inputs.turretMotorTemp = motorTemp.getValueAsDouble();
  }

  /* -------------------------------------------------------------------------- */
  /*                                CONTROL API                                 */
  /* -------------------------------------------------------------------------- */

  @Override
  public void setTurretAngle(double turretRotations) {
    // mmRequest takes mechanism rotations (turret rotations) directly.
    // Phoenix converts to motor rotations internally using TOTAL_RATIO.
    turretMotor.setControl(mmRequest.withPosition(turretRotations));
  }

  @Override
  public void setSpeed(double output) {
    turretMotor.setControl(dutyRequest.withOutput(output));
  }

  @Override
  public void setVolts(double volts) {
    turretMotor.setVoltage(volts);
  }

  @Override
  public double getTurretAngle() {
    motorPosition.refresh();
    // Already in turret rotations due to SensorToMechanismRatio — do NOT divide again.
    return motorPosition.getValueAsDouble();
  }

  /**
   * Call this when the turret is physically pointing forward (zero position). Resets the encoder to
   * 0.0 regardless of what it currently reads. Use this during teleop to recover from a bad boot
   * seed.
   *
   * <p>NOTE: After calling this, also command setTurretAngle(0.0) to prevent MotionMagic from
   * immediately driving to a stale target position.
   */
  @Override
  public void rezeroTurret() {
    turretMotor.setPosition(0.0);
    Logger.recordOutput("turret/rezero/triggered", true);
    DriverStation.reportWarning("Turret manually re-zeroed to forward position.", false);
  }

  @Override
  public void zeroTurret() {
    turretMotor.setControl(mmRequest.withPosition(0));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    turretMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    turretMotor.getConfigurator().apply(motorConfig);
  }
}
