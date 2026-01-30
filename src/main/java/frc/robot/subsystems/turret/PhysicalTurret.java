// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

/** Add your docs here. */

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterInterface;

/** Add your docs here. */
public class PhysicalTurret implements ShooterInterface {

  private final TalonFX turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);

  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new MotionMagicTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

  // TODO: Add configs
  public PhysicalTurret() {
    turretMotor.getConfigurator().apply(turretConfig);

  }
}