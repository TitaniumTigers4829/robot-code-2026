// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hublocking;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public Rotation2d heading;
  public double turretAngleToHub;
  public Translation2d hubPos;
  public double turretToHubYDist;
  public double turretToHubXDist;

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
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      hubPos = FieldConstants.RED_HUB_CENTER;
    }
    else {
      hubPos = FieldConstants.BLUE_HUB_CENTER;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets the heading of the robot as a Rotation2d
    heading = swerveDrive.getOdometryRotation2d();

    // Gets the position of the turret
    Translation2d turretPos = swerveDrive
      .getEstimatedPose()
      .getTranslation()
      .plus(
        TurretConstants.TURRET_OFFSET.rotateBy(heading));
    
    //Our turret angling math works as follows. Assuming the 0 rotations on the turret is
    //facing the front of the robot and the turret rotates positively counterclockwise, we can approximate
    //the angle it needs to turn in rotations from 0 to the target angle. This is the desired heading
    //as it doesn't factor in the current angle. With our Translation2d of the turret, we can subtract the angle 
    //that atan makes with the turret and the hub from our zero angle, which is the current heading of the robot.
    //Since ccw is positive, we subract the angle which is retrieved in degrees from 360. This will get the radians needed
    //to turn if the turret's 0 angle is the front of the robot.

    // Gets y and x distances of the turret to the hub
    turretToHubYDist = hubPos.getY() - turretPos.getY();
    turretToHubXDist = hubPos.getX() - turretPos.getX();

    // Gets the needed angle for the turret to turn to face the hub in radians
    turretAngleToHub = Math.toRadians(360 - (heading.getDegrees() - Math.toDegrees(Math.atan2(turretToHubYDist, turretToHubXDist))));

    // Converts radians to rotations
    desiredHeading = turretAngleToHub/(2*Math.PI);

    // Sets the turret to that angle
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
