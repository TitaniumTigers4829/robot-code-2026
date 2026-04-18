package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleInterface {

  @AutoLog
  class ModuleInputs {
    public boolean isTurnConnected = false;
    public boolean isDriveConnected = false;
    public boolean isEncoderConnected = false;

    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double drivePosition = 0.0;
    public double driveTorqueCurrent = 0.0;
    public double driveDesiredPosition = 0.0;
    public double driveError = 0.0;

    public double turnAbsolutePosition = 0.0;
    public double turnVelocity = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTorqueCurrent = 0.0;
    public double turnDesiredPosition = 0.0;
    public double turnError = 0.0;
  }

  /**
   * Updates the inputs created in ModuleInputs
   *
   * @param inputs the inputs to update
   */
  default void updateInputs(ModuleInputs inputs) {}

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   *
   * @param desiredState Desired state with speed and angle.
   */
  default void setDesiredState(SwerveModuleState desiredState) {}

  /**
   * Sets voltage of the drive motor for the module.
   *
   * @param voltage Voltage to set the drive motor to.
   */
  default void setDriveVoltage(Voltage voltage) {}

  /**
   * Sets voltage of the turn motor for the module.
   *
   * @param voltage Voltage to set the turn motor to.
   */
  default void setTurnVoltage(Voltage voltage) {}

  /**
   * Sets the current of the drive motor.
   *
   * @param current the current to set
   */
  default void setDriveCurrent(Current current) {}

  /**
   * Sets the current of the turn motor.
   *
   * @param current the current to set
   */
  default void setTurnCurrent(Current current) {}

  /** Stops the motors in the module. */
  default void stopModule() {}

  /**
   * Gets the current turn position of the module.
   *
   * @return The current turn position of the module.
   */
  default double getTurnRotations() {
    return 0.0;
  }

  /**
   * Gets the drive position using the internal encoder
   *
   * @return the drive position in radians
   */
  default double getDrivePositionRadians() {
    return 0.0;
  }

  default double getDistanceFromAllianceHub() {
    return 0.0;
  }

  /**
   * Sets the drive motors' pid gains
   *
   * @param kP the kP gain to set
   * @param kI the kI gain to set
   * @param kD the kD gain to set
   */
  default void setDrivePID(double kP, double kI, double kD) {}

  /**
   * Sets the turn motors' pid gains
   *
   * @param kP the kP gain to set
   * @param kI the kI gain to set
   * @param kD the kD gain to set
   */
  default void setTurnPID(double kP, double kI, double kD) {}

  /**
   * Sets the drive motors' feedforward gains
   *
   * @param kS the kS gain to set
   * @param kV the kV gain to set
   * @param kA the kA gain to set
   */
  default void setDriveFF(double kS, double kV, double kA) {}

  /**
   * Sets the turn motors' feedforward gains
   *
   * @param kS the kS gain to set
   * @param kV the kV gain to set
   * @param kA the kA gain to set
   */
  default void setTurnFF(double kS, double kV, double kA) {}
}
