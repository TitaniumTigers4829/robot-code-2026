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
import frc.robot.Constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class PhysicalTurret implements TurretInterface {

  /* -------------------------------------------------------------------------- */
  /*                                  HARDWARE                                  */
  /* -------------------------------------------------------------------------- */

  private final TalonFX turretMotor =
      new TalonFX(TurretConstants.TURRET_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);

  private final CANcoder turretEncoder = new CANcoder(TurretConstants.TURRET_CANCODER_ID);

  /* -------------------------------------------------------------------------- */
  /*                                CONFIG OBJECTS                              */
  /* -------------------------------------------------------------------------- */

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0.0);

  private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);

  /* -------------------------------------------------------------------------- */
  /*                                GEAR RATIOS                                 */
  /* -------------------------------------------------------------------------- */

  private static final double STAGE_ONE_RATIO = 16.66666;
  private static final double STAGE_TWO_RATIO = 21.6;

  private static final double TOTAL_RATIO = STAGE_ONE_RATIO * STAGE_TWO_RATIO;

  /* -------------------------------------------------------------------------- */
  /*                                STATUS SIGNALS                              */
  /* -------------------------------------------------------------------------- */

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Angle> cancoderPosition;

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

    seedFalconFromAbsolute();

    motorPosition.setUpdateFrequency(250);
    motorVelocity.setUpdateFrequency(250);

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage, motorCurrent, motorTemp);

    ParentDevice.optimizeBusUtilizationForAll(turretMotor, turretEncoder);
  }

  /* -------------------------------------------------------------------------- */
  /*                               CONFIGURATION                                */
  /* -------------------------------------------------------------------------- */

  private void configureEncoder() {
    encoderConfig.MagnetSensor.MagnetOffset = TurretConstants.ANGLE_ZERO;

    encoderConfig.MagnetSensor.SensorDirection = TurretConstants.ENCODER_REVERSED;

    turretEncoder.getConfigurator().apply(encoderConfig);
  }

  private void configureMotor() {

    motorConfig.Feedback.SensorToMechanismRatio = TOTAL_RATIO;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

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

  private void seedFalconFromAbsolute() {

    cancoderPosition.refresh();
    motorPosition.refresh();

    double absoluteTurretRot = cancoderPosition.getValueAsDouble(); // 0–1

    double motorRot = motorPosition.getValueAsDouble();

    double motorTurretRot = motorRot / TOTAL_RATIO;

    double nearestAlignedTurretRot = Math.floor(motorTurretRot) + absoluteTurretRot;

    turretMotor.setPosition(nearestAlignedTurretRot * TOTAL_RATIO);
    Logger.recordOutput("turret/positionCurrent", nearestAlignedTurretRot * TOTAL_RATIO);
  }

  /* -------------------------------------------------------------------------- */
  /*                                 IO UPDATE                                  */
  /* -------------------------------------------------------------------------- */

  public void updateInputs(TurretInputs inputs) {

    BaseStatusSignal.refreshAll(
        motorPosition, motorVelocity, motorVoltage, motorCurrent, motorTemp);

    inputs.turretAngle = motorPosition.getValueAsDouble() / TOTAL_RATIO;

    inputs.turretAngularVelocity = motorVelocity.getValueAsDouble() / TOTAL_RATIO;

    inputs.turretMotorAppliedVoltage = motorVoltage.getValueAsDouble();

    inputs.turretStatorCurrent = motorCurrent.getValueAsDouble();

    inputs.turretMotorTemp = motorTemp.getValueAsDouble();
  }

  /* -------------------------------------------------------------------------- */
  /*                                CONTROL API                                 */
  /* -------------------------------------------------------------------------- */

  public void setTurretAngle(double turretRotations) {
    turretMotor.setControl(mmRequest.withPosition(turretRotations));
  }

  public void openLoop(double output) {
    turretMotor.setControl(dutyRequest.withOutput(output));
  }

  public void setVolts(double volts) {
    turretMotor.setVoltage(volts);
  }

  public double getTurretAngle() {
    motorPosition.refresh();
    return motorPosition.getValueAsDouble() / TOTAL_RATIO;
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
