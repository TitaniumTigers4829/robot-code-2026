// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hublocking;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HubLockTurret extends Command {
  SwerveDrive swerveDrive;
  TurretSubsystem turretSubsystem;
  public double desiredHeading;
  public Translation2d hubPos;

  private final ProfiledPIDController turnController =
      new ProfiledPIDController(
          TurretConstants.TURRET_P,
          TurretConstants.TURRET_I,
          TurretConstants.TURRET_D,
          TurretConstants.TURRET_CONSTRAINTS);

  /** Creates a new HubLockTurret. */
  public HubLockTurret(SwerveDrive swerveDrive, TurretSubsystem turretSubsystem) {
    this.swerveDrive = swerveDrive;
    this.turretSubsystem = turretSubsystem;
    addRequirements(swerveDrive, turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // sets alliance to red
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      hubPos = FieldConstants.RED_HUB_CENTER;
    }
    else {
      hubPos = FieldConstants.BLUE_HUB_CENTER;
    }
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotPos = swerveDrive.getEstimatedPose().getTranslation();

    // Someone check my logic here
    desiredHeading =
        Math.atan2(((robotPos.getY() - hubPos.getY())-TurretConstants.Y_DISTANCE),
        ((robotPos.getX() - hubPos.getX())-TurretConstants.X_DISTANCE));
        
    turretSubsystem.setTurretAngle(desiredHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
