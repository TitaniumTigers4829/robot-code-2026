// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final StatusSignal<Angle> cancoderPosition;

  // TODO(second-cancoder): Uncomment when second CANcoder is installed.
  // private final StatusSignal<Angle> cancoderPosition2;

  /* -------------------------------------------------------------------------- */
  /*                                 CONSTRUCTOR                                */
  /* -------------------------------------------------------------------------- */

  public PhysicalTurret() {

    configureEncoder();
    configureMotor();

    motorPosition = turretEncoder.getPosition();
    motorVelocity = turretMotor.getVelocity();
    cancoderPosition = turretEncoder.getAbsolutePosition();

    // TODO(second-cancoder): Uncomment when second CANcoder is installed.
    // cancoderPosition2 = turretEncoder2.getAbsolutePosition();

    motorPosition.setUpdateFrequency(250);
    motorVelocity.setUpdateFrequency(250);

    // TODO(second-cancoder): Add turretEncoder2 to optimizeBusUtilizationForAll.
    ParentDevice.optimizeBusUtilizationForAll(turretMotor, turretEncoder);
  }

  /* -------------------------------------------------------------------------- */
  /*                               CONFIGURATION                                */
  /* -------------------------------------------------------------------------- */

  private void configureEncoder() {
    encoderConfig.MagnetSensor.MagnetOffset = -TurretConstants.ANGLE_ZERO;
    encoderConfig.MagnetSensor.SensorDirection = TurretConstants.ENCODER_REVERSED;
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
    motorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ANGLE;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ANGLE;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

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
  /*                                 IO UPDATE                                  */
  /* -------------------------------------------------------------------------- */

  @Override
  public void updateInputs(TurretInputs inputs) {

    // BaseStatusSignal.refreshAll(
    //     motorPosition, motorVelocity, motorVoltage, motorCurrent, motorTemp);

    motorPosition.refresh();
    motorVelocity.refresh();
    cancoderPosition.refresh();

    // Phoenix 6 already divides by SensorToMechanismRatio (TOTAL_RATIO) internally.
    // These values are already in turret output rotations — do NOT divide again.
    inputs.turretAngle = motorPosition.getValueAsDouble();
    inputs.turretAngularVelocity = motorVelocity.getValueAsDouble();
    SmartDashboard.putNumber("abs pos", cancoderPosition.getValueAsDouble());
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
    turretEncoder.setPosition(0.0);
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
