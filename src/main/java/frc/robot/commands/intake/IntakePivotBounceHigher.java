// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePivotBounceHigher extends Command {
  /** Creates a new IntakePivotBounceHigher. */
  IntakeSubsystem intakeSubsystem;

  private int i = 0;

  public IntakePivotBounceHigher(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i++;
    if (i % 20 < 10) {
      intakeSubsystem.setIntakeAngle(IntakeConstants.PIVOT_UP_POSITION);
    } else {
      intakeSubsystem.setIntakeAngle(IntakeConstants.PIVOT_BOUNCE_HIGHER_POSITION);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO maybe don't do this if the intake tries to fall down
    intakeSubsystem.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
