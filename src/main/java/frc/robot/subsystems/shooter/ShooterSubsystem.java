// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {
  private ShooterInterface shooterInterface;
  private ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
 
  private static final LoggedTunableNumber flywheelS =
      new LoggedTunableNumber("Shooter/ShooterS");
  private static final LoggedTunableNumber flywheelV =
      new LoggedTunableNumber("Shooter/ShooterV");
  private static final LoggedTunableNumber flywheelA =
      new LoggedTunableNumber("Shooter/ShooterA");
  private static final LoggedTunableNumber flywheelP =
      new LoggedTunableNumber("Shooter/ShooterP");
  private static final LoggedTunableNumber flywheelI =
      new LoggedTunableNumber("Shooter/ShooterI");
  private static final LoggedTunableNumber flywheelD =
      new LoggedTunableNumber("Shooter/ShooterD");


  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT -> {
        flywheelS.initDefault(ShooterConstants.FLYWHEEL_S);
        flywheelV.initDefault(ShooterConstants.FLYWHEEL_V);
        flywheelA.initDefault(ShooterConstants.FLYWHEEL_A);
        flywheelP.initDefault(ShooterConstants.FLYWHEEL_P);
        flywheelI.initDefault(ShooterConstants.FLYWHEEL_I);
        flywheelD.initDefault(ShooterConstants.FLYWHEEL_D);
    }
  }
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ShooterInterface shooterInterface) {
    this.shooterInterface = shooterInterface;
  }

  public double getFlywheelRPM() {
    return shooterInterface.getFlywheelRPM();
  }


  public double getVolts() {
    return shooterInterface.getVolts();
  }

  public void setFlywheelRPM(double targetRPM) {
    shooterInterface.setFlywheelRPM(targetRPM);
  }

  public double getFlywheelVelocity() {
    return inputs.flywheelVelocity;
  }

  public void setVolts(double volts) {
    shooterInterface.setVolts(volts);
  }

  public void openLoop(double output) {
    shooterInterface.openLoop(output);
  }

  public void setPercentOutput(double output) {
    shooterInterface.setPercentOutput(output);
  }

  public boolean isTopAtSetpointRPM(double targetRPM) {
    return Math.abs(targetRPM - inputs.flywheelRPM)
        < ShooterConstants.FLYWHEEL_ERROR_TOLERANCE;
  }


  @Override
  public void periodic() {
    shooterInterface.updateInputs(inputs);
    Logger.processInputs("Shooter/", inputs);

    // Update tunable numbers
    if (flywheelS.hasChanged(hashCode())
        || flywheelV.hasChanged(hashCode())
        || flywheelA.hasChanged(hashCode())) {
      shooterInterface.setFF(flywheelS.get(), flywheelV.get(), flywheelA.get());
    }
    if (flywheelP.hasChanged(hashCode())
        || flywheelI.hasChanged(hashCode())
        || flywheelD.hasChanged(hashCode())) {
      shooterInterface.setPID(flywheelP.get(), flywheelI.get(), flywheelD.get());
    }
  }
  }
