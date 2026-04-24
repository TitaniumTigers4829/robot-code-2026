package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.adjustableHood.AdjustableHoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class PassFuelCommand extends Command {
  private final TurretSubsystem turret;
  private final ShooterSubsystem shooter;
  private final AdjustableHoodSubsystem hood;
  private final SwerveDrive drive;
  private final BooleanSupplier overridingHood;

  Pose2d robotPose;
  double targetX;
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
  boolean isAimingProperly = false;

  // public SingleLinearInterpolator timeInAirLookupTable =
  //     new SingleLinearInterpolator(
  //         new double[][] {
  //           {1.5, 1.025},
  //           {2.0, 1.14},
  //           // {2.5, 1.24},
  //           {3.0, 1.3},
  //           {3.5, 1.37},
  //           {4.0, 1.4},
  //           {4.5, 1.42},
  //           {5.0, 1.45},
  //           {6, 2},
  //           {10, 3}
  //         });

  public PassFuelCommand(
      SwerveDrive drive,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      AdjustableHoodSubsystem hood,
      BooleanSupplier overridingHood) {
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    this.overridingHood = overridingHood;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetX = 14;
    } else {
      targetX = 3;
    }
  }

  @Override
  public void execute() {
    dampener = -1;

    robotPose = drive.getEstimatedPose();
    turretPose =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());

    velocityOmega = drive.getChassisSpeeds().omegaRadiansPerSecond;

    double targetY = robotPose.getY() < FieldConstants.FIELD_WIDTH_METERS / 2 ? 1.6 : 6.3;

    Translation2d targetPosition = new Translation2d(targetX, targetY);

    iterativeDistance = turretPose.getDistance(targetPosition);

    // for (int i = 0; i < 20; i++) {
    //   tAir = timeInAirLookupTable.getLookupValue(iterativeDistance);

    //   velocityXOffset = fieldRelative.vxMetersPerSecond * tAir * dampener;
    //   velocityYOffset = fieldRelative.vyMetersPerSecond * tAir * dampener;

    //   omegaXOffset =
    //       -velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getY() * tAir;
    //   omegaYOffset =
    //       velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getX() * tAir;

    //   offsettedTarget =
    //       new Pose2d(
    //           targetPosition.getX() - velocityXOffset - omegaXOffset,
    //           targetPosition.getY() - velocityYOffset - omegaYOffset,
    //           new Rotation2d());

    //   iterativeDistance = offsettedTarget.getTranslation().getDistance(turretPose);
    // }

    // distance = offsettedTarget.getTranslation().getDistance(turretPose);

    // deltaX = offsettedTarget.getX() - turretPose.getX();
    // deltaY = offsettedTarget.getY() - turretPose.getY();

    // double turretAngleRad = Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians();
    // // Wrap to [-pi, pi]
    // turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));
    // double desiredHeading = turretAngleRad / (2.0 * Math.PI);

    // desiredHeading -= 0.25; // .25 is because we zero it facing left instead of forward

    // // turret.setTurretAngle(desiredHeading);
    // // if (Math.abs(desiredHeading * TurretConstants.CANCODER_TO_TURRET -
    // turret.getTurretAngle())
    // //     < .1) {
    // //   isAimingProperly = true;
    // // } else {
    // //   isAimingProperly = false;
    // // }

    // // if (isAimingProperly) {
    // //   shooter.setPercentOutput(distance, false);
    // // } else {
    // //   shooter.stopShoot();
    // // }
    shooter.passFuel();

    // if (this.overridingHood.getAsBoolean()) {
    //   shooter.setRollerSpeed(0);
    //   shooter.stopShoot();
    // } else {

    // hood.setAngleWithoutDist(0.5);

    //   }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShoot();
    // hood.setAngleWithoutDist(0);
  }
}
