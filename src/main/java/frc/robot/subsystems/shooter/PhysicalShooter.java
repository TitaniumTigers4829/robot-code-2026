// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

/** Add your docs here. */
public class PhysicalShooter implements ShooterInterface {

  private final TalonFX flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID);

  private final SingleLinearInterpolator flywheelRPMLookupValues;

  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  private final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

  // TODO: Add configs
  public PhysicalShooter() {
    flywheelMotor.getConfigurator().apply(flywheelConfig);

    flywheelRPMLookupValues =
        new SingleLinearInterpolator(ShooterConstants.DISTANCE_TO_FLYWHEEL_RPM);
  }
}
