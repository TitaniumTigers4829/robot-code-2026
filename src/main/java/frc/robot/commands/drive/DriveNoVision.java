// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveNoVision extends Command {
  /** Creates a new DriveNoVision. */
  private final DoubleSupplier leftJoystickX, leftJoystickY, rightJoystickX;

  private final BooleanSupplier isFieldRelative, isHighRotation;
  private double angularSpeed;
  private SwerveDrive driveSubsystem;

  public DriveNoVision(
      SwerveDrive driveSubsystem,
      DoubleSupplier leftJoystickX,
      DoubleSupplier leftJoystickY,
      DoubleSupplier rightJoystickX,
      BooleanSupplier isFieldRelative,
      BooleanSupplier isHighRotation,
      Consumer<Boolean> isAligned) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    this.leftJoystickY = leftJoystickY;
    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.isFieldRelative = isFieldRelative;
    this.isHighRotation = isHighRotation;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Most of the time the driver prefers that the robot rotates slowly, as it gives them more
    // control
    // but sometimes (e.g. when fighting defense bots) being able to rotate quickly is necessary
    if (isHighRotation.getAsBoolean()) {
      angularSpeed = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    } else {
      angularSpeed = DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND;
    }

    // Drives the robot by scaling the joystick inputs
    driveSubsystem.drive(
        leftJoystickX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
        leftJoystickY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
        rightJoystickX.getAsDouble() * angularSpeed,
        isFieldRelative.getAsBoolean());
    // Runs all the code from DriveCommand that estimates pose
  }

  @Override
  public void end(boolean interrupted) {
    angularSpeed = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
