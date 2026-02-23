// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hublocking;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.adjustableHood.AdjustableHoodSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HubLockHood extends Command {

  SwerveDrive swerveDrive;
  AdjustableHoodSubsystem hoodSubsystem;
  public Rotation2d heading;
  public Translation2d hubPos;
  public double distance;
  public double turretToHubDist;

  public HubLockHood(SwerveDrive swerveDrive, AdjustableHoodSubsystem hoodSubsystem) {
    this.swerveDrive = swerveDrive;
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(swerveDrive, hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    hubPos = FieldConstants.RED_HUB_CENTER;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    heading = swerveDrive.getOdometryRotation2d();

    Translation2d turretPos =
        swerveDrive
            .getEstimatedPose()
            .getTranslation()
            .plus(TurretConstants.TURRET_OFFSET.rotateBy(heading));

    turretToHubDist = turretPos.getDistance(hubPos);

    hoodSubsystem.setHoodAngle(turretToHubDist);
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
