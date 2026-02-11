// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

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
public class PhysicalSpindexer implements SpindexerInterface {

  private final TalonFX spindexerMotor =
      new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_ID);

  private final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();


  // TODO: Add configs
  public PhysicalSpindexer() {

    spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    spindexerMotor.getConfigurator().apply(spindexerConfig);
  }

  public void setSpeed(double speed) {
    spindexerMotor.set(speed);
  }
}
