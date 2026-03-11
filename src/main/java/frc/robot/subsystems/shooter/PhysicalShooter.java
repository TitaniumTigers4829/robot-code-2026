// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

/** Add your docs here. */
public class PhysicalShooter implements ShooterInterface {

  private final TalonFX leaderFlywheelMotor =
      new TalonFX(ShooterConstants.LEADER_FLYWHEEL_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);
  private final TalonFX followerFlywheelMotor =
      new TalonFX(
          ShooterConstants.FOLLOWER_FLYWHEEL_MOTOR_ID, HardwareConstants.RIO_CAN_BUS_STRING);
  private final TalonFX kickerMotor = new TalonFX(ShooterConstants.KICKER_MOTOR_ID);
  private final TalonFX spindexerMotor = new TalonFX(ShooterConstants.SPINDEXER_MOTOR_ID);
  MotorAlignmentValue motorAlignment = MotorAlignmentValue.Opposed;

  private final SingleLinearInterpolator flywheelRPMLookupValues;

  private final VelocityVoltage rpsRequest = new VelocityVoltage(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  private final TalonFXConfiguration leaderFlywheelConfig = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> currentRPS;

  // private final TalonFXConfiguration followerFlywheelConfig = new TalonFXConfiguration();

  // TODO: Add configs, and make slav
  public PhysicalShooter() {

    leaderFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
    leaderFlywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_P;
    leaderFlywheelConfig.Slot0.kI = ShooterConstants.FLYWHEEL_I;
    leaderFlywheelConfig.Slot0.kD = ShooterConstants.FLYWHEEL_D;
    leaderFlywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_S;
    leaderFlywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_V;
    leaderFlywheelConfig.Slot0.kA = ShooterConstants.FLYWHEEL_A;

    leaderFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    followerFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));

    flywheelRPMLookupValues =
        new SingleLinearInterpolator(ShooterConstants.DISTANCE_TO_FLYWHEEL_RPM);

    currentRPS = leaderFlywheelMotor.getVelocity();

    currentRPS.setUpdateFrequency(100);
    ParentDevice.optimizeBusUtilizationForAll(leaderFlywheelMotor);
  }

  public void updateInputs(ShooterInputs inputs) {
    BaseStatusSignal.refreshAll(currentRPS);

    inputs.flywheelRPS = currentRPS.getValueAsDouble();
  }

  public double getFlywheelRPS() {
    currentRPS.refresh();
    return currentRPS.getValueAsDouble();
  }

  // test
  public void setPercentOutput(double distance) {
    leaderFlywheelMotor.setControl(
        rpsRequest.withVelocity(flywheelRPMLookupValues.getLookupValue(distance)));
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));
    new WaitCommand(0.5);
    spindexerMotor.setControl(rpsRequest.withVelocity(50));
    kickerMotor.set(0.5);
  }

  public void passFuel(double rps) {
    spindexerMotor.setControl(rpsRequest.withVelocity(50));
    kickerMotor.setControl(rpsRequest.withVelocity(50));
    leaderFlywheelMotor.setControl(rpsRequest.withVelocity(rps));
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));
  }

  public void setSpeed(double rps) {
    leaderFlywheelMotor.setControl(rpsRequest.withVelocity(rps));
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));
  }

  public void stopShoot() {
    leaderFlywheelMotor.set(0);
    followerFlywheelMotor.set(0);
    kickerMotor.set(0);
    spindexerMotor.set(0);
  }
}
