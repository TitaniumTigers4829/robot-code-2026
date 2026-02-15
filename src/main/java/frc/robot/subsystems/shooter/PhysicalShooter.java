// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

/** Add your docs here. */
public class PhysicalShooter implements ShooterInterface {

  private final TalonFX leaderFlywheelMotor =
      new TalonFX(ShooterConstants.LEADER_FLYWHEEL_MOTOR_ID);
  private final TalonFX followerFlywheelMotor =
      new TalonFX(ShooterConstants.FOLLOWER_FLYWHEEL_MOTOR_ID);
  MotorAlignmentValue motorAlignment;

  private final SingleLinearInterpolator flywheelRPMLookupValues;

  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  private final TalonFXConfiguration leaderFlywheelConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration followerFlywheelConfig = new TalonFXConfiguration();

  // TODO: Add configs
  public PhysicalShooter() {

    leaderFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    followerFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);

    flywheelRPMLookupValues =
        new SingleLinearInterpolator(ShooterConstants.DISTANCE_TO_FLYWHEEL_RPM);
  }

  public void shootFuel(double speed) {
    leaderFlywheelMotor.set(speed);
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));
  }
}
