// basically copied from 6328

package frc.robot.extras.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class AllianceFlipper {
  public static boolean isBlue() {
    return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue);
  }

  public static boolean isRed() {
    return !isBlue();
  }

  /**
   * @param translation The Translation2d to flip
   * @return Translation2d object with its x coordinate flipped over the y-axis
   */
  public static final Translation2d flipTranslation(Translation2d translation) {
    return new Translation2d(
        FieldConstants.FIELD_LENGTH_METERS - translation.getX(), translation.getY());
  }

  /**
   * @param translation The Translation to flip
   * @return Translation3d object with its x coordinate flipped over the y-axis
   */
  public static final Translation3d flipTranslation(Translation3d translation) {
    return new Translation3d(
        FieldConstants.FIELD_LENGTH_METERS - translation.getX(),
        translation.getY(),
        translation.getZ());
  }

  /**
   * @param rotation The Rotation2d to flip
   * @return Rotation2d object flipped over the y-axis
   */
  public static final Rotation2d flipRotation(Rotation2d rotation) {
    return new Rotation2d(-rotation.getCos(), rotation.getSin());
  }

  /**
   * @param rotation The Rotation3d to flip
   * @return Rotation3d object flipped over the y-axis
   */
  public static final Rotation3d flipRotation(Rotation3d rotation) {
    return new Rotation3d(
        rotation.getX(),
        rotation.getY(),
        flipRotation(Rotation2d.fromRadians(rotation.getZ())).getRadians());
  }

  /**
   * @param pose The Pose2d object to flip
   * @return The pose object with its Translation2d and Rotation2d flipped over the y-axis
   */
  public static final Pose2d flipPose(Pose2d pose) {
    return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
  }

  /**
   * @param pose The Pose3d object to flip
   * @return The pose object with its Translation3d and Rotation3d flipped over the y-axis
   */
  public static final Pose3d flipPose(Pose3d pose) {
    return new Pose3d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
  }

  /**
   * @param transform The Transfor2d object to flip
   * @return The transform object with its Translation2d and Rotation2d flipped over the y-axis
   */
  public static final Transform2d flipTransform(Transform2d transform) {
    return new Transform2d(
        flipTranslation(transform.getTranslation()), flipRotation(transform.getRotation()));
  }

  /**
   * @param transform The Transform3d object to flip
   * @return The transform object with its Translation3d and Rotation3d flipped over the y-axis
   */
  public static final Transform3d flipTransform(Transform3d transform) {
    return new Transform3d(
        flipTranslation(transform.getTranslation()), flipRotation(transform.getRotation()));
  }
}
