package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.logging.RuntimeLog;
import frc.robot.extras.math.mathutils.GearRatio;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.sim.configs.SimGyroConfig;
import frc.robot.sim.configs.SimMechanismConfig;
import frc.robot.sim.configs.SimSwerveConfig;
import frc.robot.sim.configs.SimSwerveModuleConfig;
import frc.robot.sim.configs.SimSwerveModuleConfig.WheelCof;
import frc.robot.sim.sim2026.RebuiltSim.RebuiltSimArena;
import frc.robot.sim.simMechanism.simSwerve.SimSwerve;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

/** Represents the simulation world. */
public class SimWorld {

  private final RebuiltSimArena arena;
  private final SimRobot<SimSwerve> simRobot;

  private final VisionSystemSim aprilTagSim;

  private final SimMechanismConfig driveMotorCfg =
      new SimMechanismConfig(new DCMotorExt(DCMotor.getFalcon500Foc(1), 1))
          .withFriction(Volts.of(ModuleConstants.DRIVE_S), Volts.of(ModuleConstants.DRIVE_S * 0.8))
          .withGearRatio(GearRatio.reduction(ModuleConstants.DRIVE_GEAR_RATIO))
          .withNoise(0.00)
          .withRotorInertia(KilogramSquareMeters.of(0.003));
  private final SimMechanismConfig steerMotorCfg =
      new SimMechanismConfig(new DCMotorExt(DCMotor.getFalcon500Foc(1), 1))
          .withFriction(Volts.of(ModuleConstants.TURN_S), Volts.of(ModuleConstants.TURN_S * 0.8))
          .withGearRatio(GearRatio.reduction(ModuleConstants.TURN_GEAR_RATIO))
          .withNoise(0.00)
          .withRotorInertia(KilogramSquareMeters.of(0.02));
  private final SimSwerveModuleConfig moduleCfg =
      new SimSwerveModuleConfig(
          driveMotorCfg,
          steerMotorCfg,
          WheelCof.BLACK_NITRILE.cof,
          ModuleConstants.WHEEL_DIAMETER_METERS / 2.0);
  private final SimSwerveConfig swerveConfig =
      new SimSwerveConfig(
          60.0,
          7.0,
          Units.inchesToMeters(30.5),
          Units.inchesToMeters(30.5),
          DriveConstants.MODULE_TRANSLATIONS,
          moduleCfg,
          SimGyroConfig.ofNavX2());

  /** Constructs a new simulation world for the 2026 REBUILT game. */
  public SimWorld() {
    // Create the arena first
    arena = new RebuiltSimArena(Seconds.of(HardwareConstants.LOOP_TIME_SECONDS), 5);

    // Create the robot
    simRobot = new SimRobot<>(arena, "User", swerveConfig, 1);

    arena.withWorld(
        world -> {
          // Check if the body is already in the world
          if (!world.containsBody(simRobot.getDriveTrain().chassis)) {
            world.addBody(simRobot.getDriveTrain().chassis);
          } else {
            RuntimeLog.debug("Robot chassis already in physics world, skipping duplicate add");
          }
        });

    aprilTagSim = new VisionSystemSim("AprilTags");
    aprilTagSim.addAprilTags(VisionConstants.FIELD_LAYOUT);

    // Set initial robot pose
    simRobot.getDriveTrain().setChassisWorldPose(new Pose2d(7, 4, new Rotation2d()), true);
  }

  /**
   * Returns the simulation arena.
   *
   * @return the simulation arena
   */
  public RebuiltSimArena arena() {
    return arena;
  }

  /**
   * Returns the simulation robot.
   *
   * @return the simulation robot
   */
  public SimRobot<SimSwerve> robot() {
    return simRobot;
  }

  /**
   * Returns the AprilTag simulation.
   *
   * @return the AprilTag simulation
   */
  public VisionSystemSim aprilTagSim() {
    return aprilTagSim;
  }

  /**
   * Updates the simulation.
   *
   * @param poseSupplier the pose supplier used to update the chassis's pose in the simulation world
   */
  public void update(Supplier<Pose2d> poseSupplier) {
    // Get the desired pose from the robot's odometry
    Pose2d desiredPose = poseSupplier.get();

    // Get the actual physics simulation pose
    Pose2d actualPose = simRobot.getDriveTrain().getChassisWorldPose();

    // Log both for comparison
    Logger.recordOutput("Simulation/DesiredPose", desiredPose);
    Logger.recordOutput("Simulation/ActualPose", actualPose);
    Logger.recordOutput(
        "Simulation/PoseError",
        desiredPose.getTranslation().getDistance(actualPose.getTranslation()));

    // Update robot simulation (applies motor forces, friction, etc.)
    arena.simulationPeriodic();

    // Update vision simulation with the ACTUAL physics pose
    aprilTagSim.update(actualPose);

    // Log chassis pose from drivetrain
    Logger.recordOutput("Odometry/ChassisPose", actualPose);
  }

  /**
   * Resets the simulation robot to a specific pose.
   *
   * @param pose The pose to reset to
   * @param resetVelocity Whether to reset velocity to zero
   */
  public void resetRobotPose(Pose2d pose, boolean resetVelocity) {
    simRobot.getDriveTrain().setChassisWorldPose(pose, resetVelocity);
  }

  /**
   * Gets the current collision state of the robot.
   *
   * @return True if the robot is colliding with anything
   */
  public boolean isRobotColliding() {
    return simRobot.getDriveTrain().chassis.isColliding();
  }

  /**
   * Gets the number of active collisions.
   *
   * @return Number of bodies the robot is colliding with
   */
  public int getCollisionCount() {
    return simRobot.getDriveTrain().chassis.getCollisionCount();
  }
}
