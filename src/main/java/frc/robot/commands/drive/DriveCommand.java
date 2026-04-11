package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.BooleanSupplier;
// import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class DriveCommand extends DriveCommandBase {

  private final SwerveDrive driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final DoubleSupplier leftJoystickX, leftJoystickY, rightJoystickX;
  private final BooleanSupplier isFieldRelative, isHighRotation, rightTrigger, leftBumper;

  private final PIDController trenchAlignPIDController = new PIDController(20, 0, 0);

  private double angularSpeed;

  /**
   * The command for driving the robot using joystick inputs.
   *
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftJoystickY The joystick input for driving forward and backwards
   * @param leftJoystickX The joystick input for driving left and right
   * @param rightJoystickX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive field relative
   * @param rightTrigger The boolean supplier for if the robot should drive with a slower speed
   */
  public DriveCommand(
      SwerveDrive driveSubsystem,
      VisionSubsystem visionSubsystem,
      DoubleSupplier leftJoystickX,
      DoubleSupplier leftJoystickY,
      DoubleSupplier rightJoystickX,
      BooleanSupplier isFieldRelative,
      BooleanSupplier isHighRotation,
      BooleanSupplier rightTrigger,
      BooleanSupplier leftBumper) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.rightTrigger = rightTrigger;
    this.leftBumper = leftBumper;
    addRequirements(driveSubsystem, visionSubsystem);

    this.leftJoystickY = leftJoystickY;
    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.isFieldRelative = isFieldRelative;
    this.isHighRotation = isHighRotation;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Most of the time the driver prefers that the robot rotates slowly, as it gives them more
    // control
    // but sometimes (e.g. when fighting defense bots) being able to rotate quickly is necessary
    if (isHighRotation.getAsBoolean()) {
      // angularSpeed = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
      angularSpeed = DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND;
    } else {
      angularSpeed = DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND;
    }

    double rotationSpeed = rightJoystickX.getAsDouble() * angularSpeed;

    // if (leftBumper.getAsBoolean()) {
    //   double currentRotation = driveSubsystem.getOdometryRotation2d().getRotations();
    //   double targetAngle = currentRotation >= 0 ? 0.25 : -.25;
    //   rotationSpeed = trenchAlignPIDController.calculate(targetAngle, currentRotation);
    // }

    if (leftBumper.getAsBoolean()) {
      double currentRotation = driveSubsystem.getOdometryRotation2d().getRotations();
      double targetAngle = currentRotation >= 0.25 || currentRotation <= -0.25 ? 0.5 : 0;
      trenchAlignPIDController.enableContinuousInput(-0.5, 0.5);
      rotationSpeed = trenchAlignPIDController.calculate(targetAngle, currentRotation);
    }

    double xSpeed = leftJoystickX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeed = leftJoystickY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND;

    if (rightTrigger.getAsBoolean()) {
      xSpeed =
          Math.max(
              -DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.15,
              Math.min(DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.15, xSpeed));
      ySpeed =
          Math.max(
              -DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.15,
              Math.min(DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.15, ySpeed));
      rotationSpeed =
          Math.max(
              -DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.15,
              Math.min(DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.15, rotationSpeed));
    }

    // Drives the robot by scaling the joystick inputs
    driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, isFieldRelative.getAsBoolean());
    // Runs all the code from DriveCommand that estimates pose
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    angularSpeed = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
