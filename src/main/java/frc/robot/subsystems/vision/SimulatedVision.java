package frc.robot.subsystems.vision;

import com.titaniumtigers4829.utils.NTUtils;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 *
 *
 * <h3>Simulates the vision system.</h3>
 *
 * <p>SimulatedVision is a class that simulates the vision system and extends {@link
 * PhysicalVision}. It uses the PhotonVision library to send simulated NetworkTables data to the
 * limelight tables.
 *
 * <p><b>See</b>: <a
 * href="https://github.com/PhotonVision/photonvision/blob/2a6fa1b6ac81f239c59d724da5339f608897c510/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java">PhotonVision
 * example</a> for an example of odometry simulation using PhotonVision.
 *
 * @author @Ishan1522
 */
public class SimulatedVision extends PhysicalVision {
  private PhotonCameraSim shooterCameraSim;
  private final VisionSystemSim visionSim;
  private final Supplier<VisionSystemSim> visionSimSupplier;

  private final int kResWidth = 1280;
  private final int kResHeight = 800;

  public SimulatedVision(Supplier<VisionSystemSim> visionSimSupplier) {
    super();
    this.visionSimSupplier = visionSimSupplier;
    this.visionSim = visionSimSupplier.get();

    if (this.visionSim == null) {
      System.err.println("WARNING: VisionSystemSim is null! Vision simulation will be disabled.");
      return;
    }

    initializeCameraSim();
  }

  /** Initializes the camera simulation */
  private void initializeCameraSim() {
    try {
      // Create simulated camera properties. These can be set to mimic your actual
      // camera.
      var cameraProperties = new SimCameraProperties();
      cameraProperties.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
      cameraProperties.setCalibError(0.35, 0.10);
      cameraProperties.setFPS(15);
      cameraProperties.setAvgLatencyMs(20);
      cameraProperties.setLatencyStdDevMs(5);

      // Create a PhotonCameraSim which will update the linked PhotonCamera's values
      // with visible targets.
      PhotonCamera camera = getSimulationCamera(Limelight.FRONT_LEFT);
      if (camera != null) {
        shooterCameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(shooterCameraSim, VisionConstants.BACK_TRANSFORM);

        // Enable the raw and processed streams. (http://localhost:1181 / 1182)
        shooterCameraSim.enableRawStream(true);
        shooterCameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        shooterCameraSim.enableDrawWireframe(true);
      } else {
        System.err.println("WARNING: Could not get simulation camera for FRONT_LEFT");
      }
    } catch (Exception e) {
      System.err.println("ERROR initializing camera simulation: " + e.getMessage());
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    try {
      // Make sure we have a valid vision sim
      if (visionSim == null) {
        inputs.hasResults = false;
        return;
      }

      // Update the vision simulation with the latest robot pose
      // This should be called from somewhere that knows the current robot pose

      // Abuse the updateInputs periodic call to update the sim
      for (Limelight limelight : Limelight.values()) {
        PhotonCamera camera = getSimulationCamera(limelight);
        if (camera != null) {
          List<PhotonPipelineResult> results = camera.getAllUnreadResults();
          if (!results.isEmpty()) {
            writeToTable(results, getLimelightTable(limelight), limelight);
          }
        }
      }

      // Call super.updateInputs last
      super.updateInputs(inputs);

    } catch (Exception e) {
      System.err.println("ERROR in SimulatedVision.updateInputs: " + e.getMessage());
      e.printStackTrace();
      inputs.hasResults = false;
    }
  }

  /**
   * Writes photonvision results to the limelight network table
   *
   * @param results The simulated photonvision pose estimation results
   * @param table The Limelight table to write to
   * @param limelight The Limelight to write for
   */
  private void writeToTable(
      List<PhotonPipelineResult> results, NetworkTable table, Limelight limelight) {

    if (table == null || results == null || results.isEmpty()) {
      return;
    }

    for (PhotonPipelineResult result : results) {
      if (result == null) continue;

      // Always set target visible flag
      table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);

      // Set latency
      if (result.metadata != null) {
        table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
      }

      // Handle multitag results if present
      if (result.getMultiTagResult().isPresent()) {
        var multiTagResult = result.getMultiTagResult().get();
        Transform3d best = multiTagResult.estimatedPose.best;

        List<Double> pose_data = new ArrayList<>();

        if (best != null) {
          pose_data.addAll(
              Arrays.asList(
                  best.getX(), // 0: X
                  best.getY(), // 1: Y
                  best.getZ(), // 2: Z,
                  best.getRotation().getX(), // 3: roll
                  best.getRotation().getY(), // 4: pitch
                  best.getRotation().getZ(), // 5: yaw
                  result.metadata != null
                      ? result.metadata.getLatencyMillis()
                      : 0.0, // 6: latency ms,
                  (double) multiTagResult.fiducialIDsUsed.size(), // 7: tag count
                  0.0, // 8: tag span
                  0.0, // 9: tag dist
                  result.hasTargets() ? result.getBestTarget().getArea() : 0.0 // 10: tag area
                  ));
        }

        // Add RawFiducials data
        // This is super inefficient but it's sim only, who cares.
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target != null) {
            pose_data.add((double) target.getFiducialId()); // 0: id
            pose_data.add(target.getYaw()); // 1: txnc
            pose_data.add(target.getPitch()); // 2: tync
            pose_data.add(target.getArea()); // 3: ta
            pose_data.add(0.0); // 4: distToCamera
            pose_data.add(0.0); // 5: distToRobot
            pose_data.add(target.getPoseAmbiguity()); // 6: ambiguity
          }
        }

        if (!pose_data.isEmpty()) {
          table
              .getEntry("botpose_wpiblue")
              .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
          table
              .getEntry("botpose_orb_wpiblue")
              .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        }
      }

      // Handle single tag results as fallback
      else if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        if (bestTarget != null) {
          // Create single tag pose data (simplified)
          List<Double> pose_data = new ArrayList<>();
          Transform3d best = bestTarget.getBestCameraToTarget();

          if (best != null) {
            pose_data.addAll(
                Arrays.asList(
                    best.getX(),
                    best.getY(),
                    best.getZ(),
                    best.getRotation().getX(),
                    best.getRotation().getY(),
                    best.getRotation().getZ(),
                    result.metadata != null ? result.metadata.getLatencyMillis() : 0.0,
                    1.0, // single tag
                    0.0,
                    0.0,
                    bestTarget.getArea()));

            // Add target info
            pose_data.add((double) bestTarget.getFiducialId());
            pose_data.add(bestTarget.getYaw());
            pose_data.add(bestTarget.getPitch());
            pose_data.add(bestTarget.getArea());
            pose_data.add(0.0); // distToCamera
            pose_data.add(0.0); // distToRobot
            pose_data.add(bestTarget.getPoseAmbiguity());

            table
                .getEntry("botpose_wpiblue")
                .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
          }
        }
      }
    }
  }

  /**
   * Gets the PhotonCamera for the given Limelight
   *
   * @param limelight The Limelight to get the camera for
   * @return A PhotonCamera object for the given Limelight
   */
  private PhotonCamera getSimulationCamera(Limelight limelight) {
    try {
      return switch (limelight) {
        case FRONT_LEFT -> VisionConstants.FRONT_LEFT;
        case FRONT_RIGHT -> VisionConstants.FRONT_RIGHT;

        default -> null;
      };
    } catch (Exception e) {
      System.err.println(
          "ERROR getting simulation camera for " + limelight + ": " + e.getMessage());
      return null;
    }
  }

  /**
   * Gets the Limelight network table
   *
   * @param limelight The Limelight to get the table for
   * @return The network table of the Limelight
   */
  private NetworkTable getLimelightTable(Limelight limelight) {
    try {
      return switch (limelight) {
        case FRONT_LEFT -> NTUtils.getLimelightNetworkTable(Limelight.FRONT_LEFT.getName());
        case FRONT_RIGHT -> NTUtils.getLimelightNetworkTable(Limelight.FRONT_RIGHT.getName());
        default -> null;
      };
    } catch (Exception e) {
      System.err.println("ERROR getting limelight table for " + limelight + ": " + e.getMessage());
      return null;
    }
  }

  @Override
  public void setOdometryInfo(double headingDegrees, double headingRateDegrees) {
    super.setOdometryInfo(headingDegrees, headingRateDegrees);
  }
}
