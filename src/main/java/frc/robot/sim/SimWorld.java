// package frc.robot.sim;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import frc.robot.Constants.HardwareConstants;
// import frc.robot.extras.math.mathutils.GearRatio;
// import frc.robot.extras.util.DCMotorExt;
// import frc.robot.sim.configs.SimGyroConfig;
// import frc.robot.sim.configs.SimMechanismConfig;
// import frc.robot.sim.configs.SimSwerveConfig;
// import frc.robot.sim.configs.SimSwerveModuleConfig;
// import frc.robot.sim.configs.SimSwerveModuleConfig.WheelCof;
// import frc.robot.sim.sim2026.RebuiltSim;
// import frc.robot.sim.simField.SimArena;
// import frc.robot.sim.simMechanism.simSwerve.SimSwerve;
// import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
// import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
// import frc.robot.subsystems.vision.VisionConstants;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;
// import org.photonvision.simulation.VisionSystemSim;

// /**
//  * Represents the simulation world.
//  *
//  * <p>The simulation world is composed of:
//  *
//  * <ul>
//  *   <li>A {@link SimArena} object that represents the simulation arena.
//  *   <li>A {@link SimRobot} object that represents the simulation robot.
//  *   <li>A {@link VisionSystemSim} object that represents the simulation vision system. *
//  *   <li>A {@link SimMechanismConfig} object that represents the simulation mechanism configuration.
//  *       These act as the drive and turn motors for the swerve modules.
//  *   <li>A {@link SimSwerveModuleConfig} object that represents the simulation swerve module
//  *       configuration, using the {@link SimMechanismConfig}.
//  *   <li>A {@link SimGyroConfig} object that represents the simulation gyro configuration.
//  *   <li>A {@link SimSwerveConfig} object that represents the simulation swerve configuration This
//  *       uses the {@link SimSwerveModuleConfig} and {@link SimGyroConfig}.
//  * </ul>
//  */
// public class SimWorld {

//   private final SimArena arena;
//   private final SimRobot<SimSwerve> simRobot;

//   private final VisionSystemSim aprilTagSim;

//   private final SimMechanismConfig driveMotorCfg =
//       new SimMechanismConfig(new DCMotorExt(DCMotor.getFalcon500Foc(1), 1))
//           .withFriction(Volts.of(ModuleConstants.DRIVE_S), Volts.of(ModuleConstants.DRIVE_S * 0.8))
//           .withGearRatio(GearRatio.reduction(ModuleConstants.DRIVE_GEAR_RATIO))
//           .withNoise(0.00)
//           .withRotorInertia(KilogramSquareMeters.of(0.003));
//   private final SimMechanismConfig steerMotorCfg =
//       new SimMechanismConfig(new DCMotorExt(DCMotor.getFalcon500Foc(1), 1))
//           .withFriction(Volts.of(ModuleConstants.TURN_S), Volts.of(ModuleConstants.TURN_S * 0.8))
//           .withGearRatio(GearRatio.reduction(ModuleConstants.TURN_GEAR_RATIO))
//           .withNoise(0.00)
//           .withRotorInertia(KilogramSquareMeters.of(0.02));
//   private final SimSwerveModuleConfig moduleCfg =
//       new SimSwerveModuleConfig(
//           driveMotorCfg,
//           steerMotorCfg,
//           WheelCof.BLACK_NITRILE.cof,
//           ModuleConstants.WHEEL_DIAMETER_METERS / 2.0);
//   private final SimSwerveConfig swerveConfig =
//       new SimSwerveConfig(
//           60.0,
//           7.0,
//           Units.inchesToMeters(30.5),
//           Units.inchesToMeters(30.5),
//           DriveConstants.MODULE_TRANSLATIONS,
//           moduleCfg,
//           SimGyroConfig.ofNavX2());

//   /** Constructs a new simulation world. */
//   public SimWorld() {
//     arena = new SimArena(Seconds.of(HardwareConstants.LOOP_TIME_SECONDS), 5);
//     simRobot = new SimRobot<>(arena, "User", swerveConfig, 1);

//     aprilTagSim = new VisionSystemSim("AprilTags");
//     aprilTagSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
//   }

//   /**
//    * Returns the simulation arena.
//    *
//    * @return the simulation arena
//    */
//   public SimArena arena() {
//     return arena;
//   }

//   /**
//    * Returns the simulation robot.
//    *
//    * @return the simulation robot
//    */
//   public SimRobot<SimSwerve> robot() {
//     return simRobot;
//   }

//   /**
//    * Returns the AprilTag simulation.
//    *
//    * @return the AprilTag simulation
//    */
//   public VisionSystemSim aprilTagSim() {
//     return aprilTagSim;
//   }

//   /**
//    * Updates the simulation.
//    *
//    * @param poseSupplier the pose supplier used to update the chassis's pose in the simulation world
//    */
//   public void update(Supplier<Pose2d> poseSupplier) {
//     robot().getDriveTrain().setChassisWorldPose(poseSupplier.get(), true);
//     arena().simulationPeriodic();

//     final Pose2d robotPose = simRobot.getDriveTrain().getChassisWorldPose();
//     aprilTagSim.update(robotPose);
//     Logger.recordOutput("Odometry/ChassisPose", robot().getDriveTrain().getChassisWorldPose());
//   }
// }
