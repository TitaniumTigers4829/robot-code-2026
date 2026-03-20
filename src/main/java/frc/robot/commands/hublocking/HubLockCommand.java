// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hublocking;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.adjustableHood.AdjustableHoodSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

public class HubLockCommand extends DriveCommandBase {

  SwerveDrive swerveDrive;
  VisionSubsystem visionSubsystem;
  TurretSubsystem turretSubsystem;
  AdjustableHoodSubsystem hoodSubsystem;
  public double desiredHeading;
  public Rotation2d heading;
  public double turretAngleToHub;
  public Translation2d hubPos;
  public double turretToHubYDist;
  public double turretToHubXDist;
  public double turretToHubDist;

  public HubLockCommand(
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      AdjustableHoodSubsystem hoodSubsystem,
      TurretSubsystem turretSubsystem) {
    super(swerveDrive, visionSubsystem);
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem, turretSubsystem);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      hubPos = FieldConstants.RED_HUB_CENTER;
    } else {
      hubPos = FieldConstants.BLUE_HUB_CENTER;
    }
    hoodSubsystem.resetHoodPID();
  }

  @Override
  public void execute() {
    // Gets the heading of the robot as a Rotation2d
    heading = swerveDrive.getOdometryRotation2d();

    // Gets the position of the turret
    Translation2d turretPos =
        visionSubsystem
            .getLastSeenPose()
            .getTranslation()
            .plus(TurretConstants.TURRET_OFFSET.rotateBy(heading));
    super.execute();
    /**
     * Our turret angling math works as follows. Assuming the 0 rotations on the turret is facing
     * the current heading of the robot and the turret rotates positively counterclockwise, we can
     * approximate the angle it needs to turn in rotations from 0 to the target angle. This is the
     * desired heading. With arctan we can calulate the angle the turret makes with the hub relative
     * to the y axis, otherwise known as the field relative angle. The y axis is horizontal and the
     * x axis is vertical from the driver station pov. We can subtract the heading (and therefore
     * the zero angle) of the robot from the field relative angle. This will get the radians needed
     * to turn to face the hub and when converted to rotations becomes the desired heading. *
     */

    // Gets y and x distances of the turret to the hub
    turretToHubYDist = hubPos.getY() - turretPos.getY();
    turretToHubXDist = hubPos.getX() - turretPos.getX();

    // Gets the needed angle for the turret to turn to face the hub in radians
    double turretAngleRad = Math.atan2(turretToHubYDist, turretToHubXDist) - heading.getRadians();

    // Wrap to [-pi, pi]
    turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));

    desiredHeading = turretAngleRad / (2.0 * Math.PI);

    // Clamp to turret limits
    desiredHeading =
        Math.max(TurretConstants.MIN_ANGLE, Math.min(TurretConstants.MAX_ANGLE, desiredHeading));

    // TODO: uncomment
    // turretSubsystem.setTurretAngle(desiredHeading);

    // Gets the actual distance from the hub, which becomes the paramenter for the lookup tables
    // of the hood and shooter
    turretToHubDist = Math.hypot(turretToHubXDist, turretToHubYDist);
    // turretPos.getDistance(hubPos);

    // Locks hood angle on hub
    hoodSubsystem.setAngleWithoutDist(0.5);
    super.execute();
    SmartDashboard.putNumber("dist", turretToHubDist);
    SmartDashboard.putNumber("Y dist", turretToHubYDist);
    SmartDashboard.putNumber("xdist", turretToHubXDist);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setSpeed(0);
    // hoodSubsystem.setSpeed(0);
    hoodSubsystem.resetHoodPID();
    hoodSubsystem.setAngleWithoutDist(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
