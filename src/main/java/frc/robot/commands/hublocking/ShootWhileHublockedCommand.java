// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hublocking;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;
import frc.robot.subsystems.adjustableHood.AdjustableHoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootWhileHublockedCommand extends Command {
  /** Creates a new ShootWhileHublockedCommand. */
  ShooterSubsystem shooterSubsystem;

  VisionSubsystem visionSubsystem;
  AdjustableHoodSubsystem hoodSubsystem;
  SwerveDrive swerveDrive;
  TurretSubsystem turretSubsystem;
  public double desiredHeading;
  public Rotation2d heading;
  public Translation2d hubPos;
  public double turretToHubYDist;
  public double turretToHubXDist;
  public double turretToHubDist;
  public double xVelocity;
  public double yVelocity;
  public SingleLinearInterpolator flywheelLookupTable;

  public ShootWhileHublockedCommand(
      ShooterSubsystem shooterSubsystem,
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      AdjustableHoodSubsystem hoodSubsystem,
      TurretSubsystem turretSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.swerveDrive = swerveDrive;
    this.hoodSubsystem = hoodSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.turretSubsystem = turretSubsystem;
    addRequirements(shooterSubsystem, hoodSubsystem, visionSubsystem, turretSubsystem);
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
        swerveDrive
            .getEstimatedPose()
            .getTranslation()
            .plus(TurretConstants.TURRET_OFFSET.rotateBy(heading));
    super.execute();

    // double initDist = turretPos.getDistance(hubPos);
    // double turretToHubYDistInit = hubPos.getY() - turretPos.getY();
    // double turretToHubXDistInit = hubPos.getX() - turretPos.getX();
    // double turretToHubDistInit = Math.hypot(turretToHubXDistInit, turretToHubYDistInit);

    // SmartDashboard.putNumber("init hub dist", turretToHubDistInit);

    // xVelocity = swerveDrive.getChassisSpeeds().vxMetersPerSecond;
    // yVelocity = swerveDrive.getChassisSpeeds().vyMetersPerSecond;

    // double chassisVelocity = Math.hypot(xVelocity, yVelocity);

    // double fullVelocity =
    //     flywheelLookupTable.getLookupValue(turretToHubDistInit) * (2 * Math.PI * 1.5)
    //         + chassisVelocity;

    // double tAir = turretToHubDistInit / fullVelocity;

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
    // Gets the needed angle for the turret to turn to face the hub in radians

    // double xOffset = xVelocity * tAir;
    // double yOffset = yVelocity * tAir;

    heading = swerveDrive.getOdometryRotation2d();

    // Gets y and x distances of the turret to the hub
    // TODO: translation2d.dist?
    turretToHubYDist = hubPos.getY() - turretPos.getY();
    turretToHubXDist = hubPos.getX() - turretPos.getX();

    // TODO: figure out lmao
    // TODO: math.atan2 is from [-pi, pi] and our heading is [0, 2pi] i think? 
    // TODO: remove the math.pi addition if so
    double turretAngleRad = (Math.PI + Math.atan2(turretToHubYDist, turretToHubXDist)) - heading.getRadians();
    // Wrap to [0, 2pi]
    // TODO: change to just math.PI if our heading is [-pi, pi]
    turretAngleRad = turretAngleRad % (2 * Math.PI);

    // Clamp to turret limits
    // TODO: look at and make sure this makes sense
    desiredHeading = map(desiredHeading, 0, 2 * Math.PI, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);

    // Gets the actual distance from the hub, which becomes the paramenter for the lookup tables
    // of the hood and shooter
    turretToHubDist = Math.hypot(turretToHubXDist, turretToHubYDist);

    SmartDashboard.putNumber("hub dist", turretToHubDist);

    hoodSubsystem.setHoodAngle(turretToHubDist);
    turretSubsystem.setTurretAngle(desiredHeading);
    // new WaitCommand(0.5);
    shooterSubsystem.setPercentOutput(turretToHubDist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShoot();
    // this will slam it into the thing because we only do the PID one time when theres a lot of
    // error rather than the whole time its getting closer to 0
    // but like its fine
    hoodSubsystem.setAngleWithoutDist(0);
    shooterSubsystem.setRollerSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * This should just map a value from [origMin, origMax] to [newMin, newMax]
   * https://www.javathinking.com/blog/convert-a-number-range-to-another-range-maintaining-ratio-java/
   * @param val value to map
   * @param origMin value min
   * @param origMax value max
   * @param newMin new min
   * @param newMax new max
   * @return the mapped value, lmao
   */
  private double map(double val, double origMin, double origMax, double newMin, double newMax) {
    return newMin + ((val - origMin) * (newMax - newMin)) / (origMax - origMin);
  }
}
