// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

/** Add your docs here. */
public class PhysicalClimb implements ClimbInterface {

  private final TalonFX climbMotor =
      new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);


  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  private final TalonFXConfiguration ClimbConfig = new TalonFXConfiguration();

  // private final TalonFXConfiguration followerClimbConfig = new TalonFXConfiguration();

  // TODO: Add configs, and make slav
  public PhysicalClimb() {

    ClimbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ClimbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ClimbConfig.Slot0.kP = ClimbConstants.CLIMB_P;
    ClimbConfig.Slot0.kI = ClimbConstants.CLIMB_I;
    ClimbConfig.Slot0.kD = ClimbConstants.CLIMB_D;
    ClimbConfig.Slot0.kS = ClimbConstants.CLIMB_S;
    ClimbConfig.Slot0.kV = ClimbConstants.CLIMB_V;
    ClimbConfig.Slot0.kA = ClimbConstants.CLIMB_A;

    climbMotor.getConfigurator().apply(ClimbConfig);
  }

  public void levelOne() {
    climbMotor.setControl(mmPositionRequest.withPosition(0));
  }
}
