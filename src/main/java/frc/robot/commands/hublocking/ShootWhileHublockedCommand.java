// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hublocking;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.adjustableHood.AdjustableHoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootWhileHublockedCommand extends Command {
  /** Creates a new ShootWhileHublockedCommand. */
  ShooterSubsystem shooterSubsystem;

  VisionSubsystem visionSubsystem;
  AdjustableHoodSubsystem hoodSubsystem;
  SwerveDrive swerveDrive;
  public double desiredHeading;
  public Rotation2d heading;
  public double turretAngleToHub;
  public Translation2d hubPos;
  public double turretToHubDist;

  public ShootWhileHublockedCommand(
      ShooterSubsystem shooterSubsystem,
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      AdjustableHoodSubsystem hoodSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.swerveDrive = swerveDrive;
    this.hoodSubsystem = hoodSubsystem;
    this.visionSubsystem = visionSubsystem;
    // addRequirements(shooterSubsystem, swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      hubPos = FieldConstants.RED_HUB_CENTER;
    } else {
      hubPos = FieldConstants.BLUE_HUB_CENTER;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    heading = swerveDrive.getOdometryRotation2d();

    // Gets the position of the turret
    Translation2d turretPos =
        visionSubsystem
            .getPoseFromAprilTags(Limelight.FRONT)
            .getTranslation()
            .plus(TurretConstants.TURRET_OFFSET.rotateBy(heading));

    // Gets the actual distance from the hub, which becomes the paramenter for the lookup tables
    // of the hood and shooter
    turretToHubDist = turretPos.getDistance(hubPos);

    shooterSubsystem.setPercentOutput(turretAngleToHub);

    // hoodSubsystem.setHoodAngle(turretAngleToHub);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
