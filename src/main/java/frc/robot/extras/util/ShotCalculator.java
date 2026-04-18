package frc.robot.extras.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

/**
 * Utility class for calculating shot parameters (distance, angle, etc.) to the target hub. Used
 * primarily for simulation visualization and aiming calculations.
 */
public class ShotCalculator {

  /**
   * Calculates the distance from the robot to its alliance's target hub.
   *
   * @param robotPose The current pose of the robot.
   * @return Distance in meters.
   */
  public static double calculateShotDistance(Pose2d robotPose) {
    Translation2d hubCenter =
        AllianceFlipper.isBlue() ? FieldConstants.BLUE_HUB_CENTER : FieldConstants.RED_HUB_CENTER;
    return robotPose.getTranslation().getDistance(hubCenter);
  }

  /**
   * Calculates the field‑relative angle (in degrees) from the robot to its alliance's target hub.
   *
   * @param robotPose The current pose of the robot.
   * @return Angle in degrees (0–360), where 0° points toward the red alliance wall.
   */
  public static double calculateShotAngle(Pose2d robotPose) {
    Translation2d hubCenter =
        AllianceFlipper.isBlue() ? FieldConstants.BLUE_HUB_CENTER : FieldConstants.RED_HUB_CENTER;
    Translation2d toHub = hubCenter.minus(robotPose.getTranslation());
    return toHub.getAngle().getDegrees();
  }

  /**
   * (Optional) Calculates the required hood angle for a given distance to the hub. Currently
   * returns a placeholder value. Replace with a lookup table or polynomial.
   *
   * @param distance Distance to the hub in meters.
   * @return Hood angle in degrees.
   */
  public static double calculateRequiredHoodAngle(double distance) {
    // TODO: Replace with actual interpolation or function.
    // Example: if (distance < 2.0) return 30.0;
    //          else if (distance < 4.0) return 25.0;
    //          else return 20.0;
    return 30.0; // placeholder
  }
}
