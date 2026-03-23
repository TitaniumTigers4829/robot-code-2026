package frc.robot.commands.hublocking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;
import frc.robot.subsystems.adjustableHood.AdjustableHoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class ShootWhileMove extends Command {
  private final TurretSubsystem turret;
  private final ShooterSubsystem shooter;
  private final AdjustableHoodSubsystem hood;
  private final SwerveDrive drive;
  Pose2d robotPose;
  Translation2d targetPosition;
  Pose2d offsettedTarget;
  Translation2d turretOffsetPose = TurretConstants.TURRET_OFFSET;
  Translation2d turretPose;
  double deltaX = 0;
  double deltaY = 0;
  double velocityXOffset = 0;
  double velocityYOffset = 0;
  double velocityOmega = 0;
  double omegaXOffset = 0;
  double omegaYOffset = 0;
  double tAir = 1; // calculate this with utility class or interpolating tree
  ChassisSpeeds fieldRelative;
  double iterativeDistance = 0;
  double distance;
  double dampener;

  public SingleLinearInterpolator timeInAirLookupTable = new SingleLinearInterpolator(new double[][]{
      {2.20, 1.3},
      {2.59, 1.26},
      {3.03, 1.3},
      {3.39, 1.4},
      {3.88, 1.5},
      {4.39, 1.6},
      {5.0, 1.66}
});

  public ShootWhileMove(
      SwerveDrive drive, TurretSubsystem turret, ShooterSubsystem shooter, AdjustableHoodSubsystem hood) {
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetPosition = FieldConstants.RED_HUB_CENTER;
    } else {
      targetPosition = FieldConstants.BLUE_HUB_CENTER;
    }
  }

  @Override
  public void execute() {
    dampener = 1;

    robotPose = drive.getEstimatedPose();
    turretPose =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());

    velocityOmega = drive.getChassisSpeeds().omegaRadiansPerSecond;

    iterativeDistance = turretPose.getDistance(targetPosition);

    for (int i = 0; i < 20; i++) {
        tAir = timeInAirLookupTable.getLookupValue(iterativeDistance);

        velocityXOffset = fieldRelative.vxMetersPerSecond * tAir * dampener;
        velocityYOffset = fieldRelative.vyMetersPerSecond * tAir * dampener;

        omegaXOffset =
            -velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getY() * tAir;
        omegaYOffset =
            velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getX() * tAir;

        offsettedTarget =
            new Pose2d(
                targetPosition.getX() - velocityXOffset - omegaXOffset,
                targetPosition.getY() - velocityYOffset - omegaYOffset,
                new Rotation2d());

        iterativeDistance = offsettedTarget.getTranslation().getDistance(turretPose);
    }

    distance = offsettedTarget.getTranslation().getDistance(turretPose);

    deltaX = offsettedTarget.getX() - turretPose.getX();
    deltaY = offsettedTarget.getY() - turretPose.getY();

    double turretAngleRad = Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians();
    // Wrap to [-pi, pi]
    turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));
    double desiredHeading = turretAngleRad / (2.0 * Math.PI);
    // TODO: I don't think this is needed
    desiredHeading =
        Math.max(TurretConstants.MIN_ANGLE, Math.min(TurretConstants.MAX_ANGLE, desiredHeading));

    turret.setTurretAngle(desiredHeading);

    shooter.setPercentOutput(distance);
    hood.setHoodAngle(distance);
    
    Logger.recordOutput("Shoot on move At Hub/Desired Hub", offsettedTarget);

    Logger.recordOutput("Shoot on move At Hub/Distance to Desire Hub", distance);

    Logger.recordOutput("Shoot on move At Hub/Desired Rotation", desiredHeading);
    Logger.recordOutput("Shoot on move At Hub/ XOffset", deltaX);
    Logger.recordOutput("Shoot on move At Hub/ YOffset", deltaY);
    Logger.recordOutput("Shoot on move At Hub/ OmegaXOffset", omegaXOffset);
    Logger.recordOutput("Shoot on move At Hub/ OmegaYOffset", omegaYOffset);
    Logger.recordOutput("Shoot on move At Hub/ LinearXOffset", velocityXOffset);
    Logger.recordOutput("Shoot on move At Hub/ LinearYOffset", velocityYOffset);
    Logger.recordOutput("Shoot on move At Hub/tAir", tAir);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShoot();
    hood.setAngleWithoutDist(0);
    shooter.setRollerSpeed(0.0);
  }
}