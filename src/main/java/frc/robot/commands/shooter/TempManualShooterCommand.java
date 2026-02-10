// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TempManualShooterCommand extends Command {
  /** Creates a new TempManualShooterCommand. */
  ShooterSubsystem shooter;

  Translation2d hubPos;
  public double distance;
  public double turretToHubDist;
  SwerveDrive swerveDrive;
  public Rotation2d heading;

  public TempManualShooterCommand(SwerveDrive swerveDrive, ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.get() == Alliance.Red) {
      hubPos = FieldConstants.RED_HUB_CENTER;
    } else {
      hubPos = FieldConstants.BLUE_HUB_CENTER;
    }
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

    shooter.setSpeed(turretToHubDist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
