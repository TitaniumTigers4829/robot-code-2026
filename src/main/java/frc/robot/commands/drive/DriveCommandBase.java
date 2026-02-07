package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.math.interpolation.MultiLinearInterpolator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionConstants;

public abstract class DriveCommandBase extends Command {

  private final MultiLinearInterpolator oneAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
  private final MultiLinearInterpolator twoAprilTagLookupTable =
      new MultiLinearInterpolator(VisionConstants.TWO_APRIL_TAG_LOOKUP_TABLE);

  // private final VisionSubsystem vision;
  private final SwerveDrive swerveDrive;

  /**
   * An abstract class that handles pose estimation while driving.
   *
   * @param driveSubsystem The subsystem for the swerve drive
   */
  public DriveCommandBase(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    // this.vision = vision;
    // It is important that you do addRequirements(driveSubsystem, vision) in whatever
    // command extends this
    // DO NOT do addRequirements here, it will break things
  }

  @Override
  public void execute() {
    swerveDrive.addPoseEstimatorSwerveMeasurement();

    //   // Update the odometry information for the vision subsystem to use while filtering the
    // vision
    //   // pose estimate
    //   vision.setOdometryInfo(
    //       swerveDrive.getOdometryRotation2d().getDegrees(), swerveDrive.getGyroRate());
    //   for (Limelight limelight : Limelight.values()) {
    //     addLimelightVisionMeasurement(limelight);
    //   }
    // }

    // private double scaleStandardDeviations(Limelight limelight, double standardDeviation) {
    //   return limelight.hasInternalIMU() ? standardDeviation : standardDeviation * 1.5;
    // }

    // /**
    //  * Calculates the pose from the limelight and adds it to the pose estimator.
    //  *
    //  * @param limelight The limelight to calculate the pose from
    //  */
    // public void addLimelightVisionMeasurement(Limelight limelight) {
    //   // Only do pose calculation if we can see the april tags
    //   if (vision.canSeeAprilTags(limelight) && vision.isValidMeasurement(limelight)) {
    //     Logger.recordOutput(
    //         "Vision/valid measurement" + limelight.getId(),
    // vision.isValidMeasurement(limelight));
    //     // Only do pose calculation if the measurement from the limelight is valid
    //     double distanceFromClosestAprilTag = vision.getLimelightAprilTagDistance(limelight);

    //     // Depending on how many april tags we see, we change our confidence as more april tags
    //     // results in a much more accurate pose estimate
    //     // So if we only see 1 april tag, we have *high* standard deviations -> lower confidence
    //     if (vision.getNumberOfAprilTags(limelight) == 1) {
    //       // But then we use the lookup table here to account for how far away the robot is from
    // the
    //       // april tag
    //       // because if we are closer to the april tag, we are more confident in our position ->
    //       // lower
    //       // standard deviation
    //       double[] standardDeviations =
    //           oneAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
    //       swerveDrive.setPoseEstimatorVisionConfidence(
    //           scaleStandardDeviations(limelight, standardDeviations[0]),
    //           scaleStandardDeviations(limelight, standardDeviations[1]),
    //           scaleStandardDeviations(limelight, standardDeviations[2]));
    //     } else if (vision.getNumberOfAprilTags(limelight) > 1) {
    //       double[] standardDeviations =
    //           twoAprilTagLookupTable.getLookupValue(distanceFromClosestAprilTag);
    //       swerveDrive.setPoseEstimatorVisionConfidence(
    //           scaleStandardDeviations(limelight, standardDeviations[0]),
    //           scaleStandardDeviations(limelight, standardDeviations[1]),
    //           scaleStandardDeviations(limelight, standardDeviations[2]));
    //     }

    //     // Adds the timestamped pose gotten from the limelights to our pose estimation
    //     swerveDrive.addPoseEstimatorVisionMeasurement(
    //         vision.getPoseFromAprilTags(limelight),
    //         TimeUtil.getLogTimeSeconds() - vision.getLatencySeconds(limelight));
    //   }
    //   Logger.recordOutput(
    //       "Vision/valid measurement" + limelight.getId(), vision.isValidMeasurement(limelight));
  }
}
