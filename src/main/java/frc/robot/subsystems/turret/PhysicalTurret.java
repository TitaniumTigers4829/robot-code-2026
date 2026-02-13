// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

/** Add your docs here. */
public class PhysicalTurret implements TurretInterface {

  private final TalonFX turretMotor =
      new TalonFX(TurretConstants.TURRET_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
  private final CANcoder turretEncoder = new CANcoder(TurretConstants.TURRET_CANCODER_ID);

  // Commented out because torqueFOC is in theory easier to tune
  // private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  // private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);


  private final StatusSignal<Voltage> turretMotorAppliedVoltage;
  private final StatusSignal<Double> desiredAngle;
  private final StatusSignal<AngularVelocity> turretAngularVelocity;
  private final StatusSignal<Angle> turretAngle;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> motorTemp;
  private final double angleError;

  private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

  private final CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();

  public PhysicalTurret() {

    turretConfig.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    turretEncoderConfig.MagnetSensor.MagnetOffset = TurretConstants.ANGLE_ZERO;
    turretEncoderConfig.MagnetSensor.SensorDirection = TurretConstants.ENCODER_REVERSED;

    turretEncoder.getConfigurator().apply(turretEncoderConfig);

    turretConfig.Slot0.kP = TurretConstants.TURRET_P;
    turretConfig.Slot0.kI = TurretConstants.TURRET_I;
    turretConfig.Slot0.kD = TurretConstants.TURRET_D;
    turretConfig.Slot0.kS = TurretConstants.TURRET_S;
    turretConfig.Slot0.kV = TurretConstants.TURRET_V;
    turretConfig.Slot0.kA = TurretConstants.TURRET_A;

    turretConfig.MotionMagic.MotionMagicAcceleration =
        TurretConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;
    turretConfig.MotionMagic.MotionMagicCruiseVelocity =
        TurretConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;

    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set current limits
    turretConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
    turretConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable =
        TurretConstants.STATOR_CURRENT_LIMIT_ENABLE;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable =
        TurretConstants.SUPPLY_CURRENT_LIMIT_ENABLE;

    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretConfig.MotorOutput.DutyCycleNeutralDeadband = TurretConstants.TURRET_DEADBAND;

    turretConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ANGLE;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ANGLE;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turretMotor.getConfigurator().apply(turretConfig);
    turretAngle = turretEncoder.getAbsolutePosition();
    turretMotorAppliedVoltage = turretMotor.getMotorVoltage();
    turretAngularVelocity = turretMotor.getVelocity();
    dutyCycle = turretMotor.getDutyCycle();
    statorCurrent = turretMotor.getStatorCurrent();
    motorTemp = turretMotor.getDeviceTemp();
    desiredAngle = turretMotor.getClosedLoopReference();
    angleError =
        turretMotor.getClosedLoopReference().getValueAsDouble()
            - turretMotor.getPosition().getValueAsDouble();

    // Higher frequency for turret angle and its change over time (angular velocity) because
    // its more important that the other signals
    turretAngle.setUpdateFrequency(250.0);
    turretAngularVelocity.setUpdateFrequency(250.0);
    desiredAngle.setUpdateFrequency(250.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, turretMotorAppliedVoltage, dutyCycle, statorCurrent, motorTemp);
    ParentDevice.optimizeBusUtilizationForAll(turretMotor, turretEncoder);
  }

  public void updateInputs(TurretInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretAngle,
        turretAngularVelocity,
        turretMotorAppliedVoltage,
        dutyCycle,
        statorCurrent,
        motorTemp,
        desiredAngle);

    inputs.turretAngle =
        turretAngle.getValueAsDouble()
            + BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                turretAngle, turretAngularVelocity);
    inputs.turretAngularVelocity = turretAngularVelocity.getValueAsDouble();
    inputs.turretMotorAppliedVoltage = turretMotorAppliedVoltage.getValueAsDouble();
    inputs.turretDutyCycle = dutyCycle.getValueAsDouble();
    inputs.turretStatorCurrent = statorCurrent.getValueAsDouble();
    inputs.turretMotorTemp = motorTemp.getValueAsDouble();
  }

  public double getTurretAngle(double angle) {
    turretAngle.refresh();
    return turretAngle.getValueAsDouble();
  }

  // Assumes facing the front of the robot is 0 rotations
  // Normalizes angle
  public void setTurretAngle(double desiredAngle) {
    if (Math.abs(desiredAngle - getTurretAngle()) < 0.5) {
      turretMotor.setControl(mmTorqueRequest.withPosition(desiredAngle));
    } else {
      turretMotor.setControl(mmTorqueRequest.withPosition(Math.abs(Math.abs(desiredAngle) - 1)));
    }
  }

  // For manual in case turret angling fucks up
  public void setSpeed(double speed) {
    turretMotor.set(speed);
  }

  public void openLoop(double output) {
    turretMotor.setControl(currentOut.withOutput(output));
  }

  public void setVolts(double volts) {
    turretMotor.setVoltage(volts);
  }

  public double getVolts() {
    return turretMotorAppliedVoltage.getValueAsDouble();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    turretConfig.Slot0.kP = kP;
    turretConfig.Slot0.kI = kI;
    turretConfig.Slot0.kD = kD;
    turretMotor.getConfigurator().apply(turretConfig);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    turretConfig.Slot0.kS = kS;
    turretConfig.Slot0.kV = kV;
    turretConfig.Slot0.kA = kA;
    turretMotor.getConfigurator().apply(turretConfig);
  }
}
