package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.logging.Tracer;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import org.littletonrobotics.junction.Logger;

/**
 * VisionSubsystem is a subsystem for managing vision-related tasks in the robot, including
 * interacting with limelights, processing vision data, and logging inputs.
 *
 * <p>It provides methods for getting detailed vision information, setting heading data for pose
 * estimation, and retrieving poses based on April Tag detection.
 *
 * @author Ishan
 */
public class VisionSubsystem extends SubsystemBase {

  private Pose2d lastSeenPose = new Pose2d();

  private final VisionInterface visionInterface;
  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

  public VisionSubsystem(VisionInterface visionInterface) {
    // Initializing Fields
    this.visionInterface = visionInterface;
  }

  @Override
  public void periodic() {
    // Updates limelight inputs
    visionInterface.updateInputs(inputs);
    Logger.processInputs("Vision/", inputs);
    Tracer.traceFunc("Vision/", () -> visionInterface.updateInputs(inputs));
  }

  /**
   * Gets the number of April Tags detected by a specified Limelight.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The number of April Tags detected by the specified Limelight.
   */
  public int getNumberOfAprilTags(Limelight limelight) {
    return inputs.limelightTargets[limelight.getId()];
  }

  /**
   * Gets the distance to the April Tags detected by a specified Limelight.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The distance to the April Tags detected, in meters.
   */
  public double getLimelightAprilTagDistance(Limelight limelight) {
    return inputs.limelightAprilTagDistances[limelight.getId()];
  }

  /**
   * Gets the timestamp of the last data received from a specified Limelight.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The timestamp of the last data in seconds.
   */
  public double getTimeStampSeconds(Limelight limelight) {
    return inputs.limelightTimestamps[limelight.getId()];
  }

  /**
   * Gets the latency of a specified Limelight in seconds.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The latency of the Limelight in seconds.
   */
  public double getLatencySeconds(Limelight limelight) {
    return inputs.limelightLatencies[limelight.getId()];
  }

  /**
   * Sets heading information for the robot. This information is used to assist in pose estimation
   * by distinguishing between multiple MegaTags.
   *
   * @param headingDegrees The robot's heading in degrees.
   * @param headingRateDegrees The robot's rate of rotation in degrees per second.
   */
  public void setOdometryInfo(double headingDegrees, double headingRateDegrees) {
    visionInterface.setOdometryInfo(headingDegrees, headingRateDegrees);
  }

  /**
   * Checks if a specified Limelight can see any April Tags.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight sees any April Tags, false otherwise.
   */
  public boolean canSeeAprilTags(Limelight limelight) {
    return inputs.limelightSeesAprilTags[limelight.getId()];
  }

  /**
   * Gets the pose of the robot calculated from the April Tags detected by a specified Limelight.
   *
   * @param limelight a limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return The pose of the robot based on the detected April Tags.
   */
  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    return inputs.limelightCalculatedPoses[limelight.getId()];
  }

  /**
   * Gets the last seen pose of the robot based on the latest data from any Limelight.
   *
   * @return The last seen pose of the robot.
   */
  public Pose2d getLastSeenPose() {
    for (Pose2d pose : inputs.megatag1PoseEstimates) {
      if (pose != null && !pose.equals(new Pose2d())) {
        lastSeenPose = pose;
      }
    }

    return lastSeenPose;
  }

  public boolean isValidMeasurement(Limelight limelight) {
    return visionInterface.isValidMeasurement(limelight);
  }
}
