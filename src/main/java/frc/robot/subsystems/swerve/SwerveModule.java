package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;
import frc.robot.extras.logging.Tracer;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

  private final ModuleInterface moduleInterface;
  private final String moduleName;
  private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();

  private final Alert driveFaultAlert;
  private final Alert encoderFaultAlert;
  private final Alert turnFaultAlert;

  private static final LoggedTunableNumber driveS = new LoggedTunableNumber("Drive/Module/DriveS");
  private static final LoggedTunableNumber driveV = new LoggedTunableNumber("Drive/Module/DriveV");
  private static final LoggedTunableNumber driveP = new LoggedTunableNumber("Drive/Module/DriveP");
  private static final LoggedTunableNumber driveD = new LoggedTunableNumber("Drive/Module/DriveD");
  private static final LoggedTunableNumber turnP = new LoggedTunableNumber("Drive/Module/TurnP");
  private static final LoggedTunableNumber turnD = new LoggedTunableNumber("Drive/Module/TurnD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT, SWERVE_ROBOT -> {
        driveS.initDefault(ModuleConstants.DRIVE_S);
        driveV.initDefault(ModuleConstants.DRIVE_V);
        driveP.initDefault(ModuleConstants.DRIVE_P);
        driveD.initDefault(ModuleConstants.DRIVE_D);
        turnP.initDefault(ModuleConstants.TURN_P);
        turnD.initDefault(ModuleConstants.TURN_D);
      }
      default -> {
        driveS.initDefault(0.014);
        driveV.initDefault(0.134);
        driveP.initDefault(0.1);
        driveD.initDefault(0);
        turnP.initDefault(10.0);
        turnD.initDefault(0);
      }
    }
  }

  public SwerveModule(ModuleInterface moduleInterface, String moduleName) {
    this.moduleInterface = moduleInterface;
    this.moduleName = moduleName;
    this.driveFaultAlert = new Alert("Drive-" + moduleName + " Hardware Fault", AlertType.kError);
    this.driveFaultAlert.set(false);

    this.encoderFaultAlert =
        new Alert("Encoder-" + moduleName + " Hardware Fault", AlertType.kError);
    this.encoderFaultAlert.set(false);

    this.turnFaultAlert = new Alert("Turn-" + moduleName + " Hardware Fault", AlertType.kError);
    this.turnFaultAlert.set(false);
  }

  /** Updates the module's odometry inputs. */
  public void updateOdometryInputs() {
    moduleInterface.updateInputs(moduleInputs);
    Logger.processInputs("Drive/Module-" + moduleName, moduleInputs);
    Tracer.traceFunc("Module-" + moduleName, () -> moduleInterface.updateInputs(moduleInputs));
    this.driveFaultAlert.set(!moduleInputs.isDriveConnected);
    this.encoderFaultAlert.set(!moduleInputs.isEncoderConnected);
    this.turnFaultAlert.set(!moduleInputs.isTurnConnected);
  }

  /**
   * Sets the voltage of the module.
   *
   * @param volts the voltage to set the module to
   */
  public void setVoltage(Voltage volts) {
    moduleInterface.setDriveVoltage(volts);
    moduleInterface.setTurnVoltage(Volts.zero());
  }

  /**
   * Sets the current for the module
   *
   * @param current the current to set the module to
   */
  public void setCurrent(Current current) {
    moduleInterface.setDriveCurrent(current);
    moduleInterface.setTurnCurrent(Amps.zero());
  }

  /**
   * Gets the drive voltage of the module.
   *
   * @return the drive voltage of the module
   */
  public double getDriveVoltage() {
    return moduleInputs.driveAppliedVolts;
  }

  /**
   * Gets the drive velocity of the module.
   *
   * @return the drive velocity of the module
   */
  public double getCharacterizationVelocity() {
    return moduleInputs.driveVelocity;
  }

  /**
   * Sets the desired state of the module. It optimizes this meaning that it will adjust the turn
   * angle to be the shortest path to the desired angle. So rather than turning 170 degrees CW it
   * will turn 10 degrees CCW and invert the motor.
   *
   * @param state
   */
  public void setOptimizedDesiredState(SwerveModuleState state) {
    state.optimize(getTurnRotation());
    if (state.speedMetersPerSecond < 0.01) {
      moduleInterface.stopModule();
    }

    moduleInterface.setDesiredState(state);
    Logger.recordOutput("Drive/desired turn angle", state.angle.getRotations());
  }

  /** Stops the module */
  public void stopModule() {
    moduleInterface.stopModule();
  }

  /**
   * Gets the turn angle of the module.
   *
   * @return the turn angle of the module 0 being forward, CCW being positive
   */
  public Rotation2d getTurnRotation() {
    return Rotation2d.fromRotations(moduleInputs.turnAbsolutePosition);
  }

  /**
   * Gets the turn velocity of the module.
   *
   * @return the turn velocity in rotations per second
   */
  public double getTurnVelocity() {
    return moduleInputs.turnVelocity;
  }

  /** Returns the current drive position of the module in meters. */
  public double getDrivePositionMeters() {
    return ModuleConstants.DRIVE_TO_METERS * moduleInputs.drivePosition;
  }

  /**
   * Gets the drive velocity of the module in meters per second.
   *
   * @return the drive velocity in meters per second
   */
  public double getDriveVelocityMetersPerSec() {
    return ModuleConstants.DRIVE_TO_METERS_PER_SECOND * moduleInputs.driveVelocity;
  }

  /**
   * Gets the drive position in radians.
   *
   * @return a double containing current position of the driving motors in radians.
   */
  public double getDrivePositionRadians() {
    return moduleInterface.getDrivePositionRadians();
  }

  /**
   * Gets the measured state of the module, which includes the drive velocity and turn rotation.
   *
   * @return a SwerveModuleState object containing the drive velocity and turn rotation
   */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSec(), getTurnRotation());
  }

  /**
   * Gets the module position consisting of the distance it has traveled and the angle it is
   * rotated.
   *
   * @return a SwerveModulePosition object containing position and rotation
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getTurnRotation());
  }

  /**
   * This is called in the periodic method of the SwerveDrive. It is used to update module values
   * periodically
   */
  public void periodic() {

    // Update tunable numbers
    if (driveS.hasChanged(hashCode()) || driveV.hasChanged(hashCode())) {
      moduleInterface.setDriveFF(driveS.get(), driveV.get(), 0.0);
    }
    if (driveP.hasChanged(hashCode()) || driveD.hasChanged(hashCode())) {
      moduleInterface.setDrivePID(driveP.get(), 0, driveD.get());
    }
    if (turnP.hasChanged(hashCode()) || turnD.hasChanged(hashCode())) {
      moduleInterface.setTurnPID(turnP.get(), 0, turnD.get());
    }
    SmartDashboard.putNumber(moduleName + "offset", getPosition().angle.getRotations());
  }
}
