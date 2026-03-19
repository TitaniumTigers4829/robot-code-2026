package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeInterface extends Subsystem {
  @AutoLog
  public static class IntakeInputs {
    public boolean isIntakeDeployed = false;
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorCurrent = 0.0;
    public double intakeAngle = 0.0;
    public double intakePivotSpeed = 0.0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setIntakeAngle(double angle) {}

  public default void intakeFuel() {}

  public default void outakeFuel() {}

  public default void setAngle(double angle) {}

  public default void setSpeed(double speed) {}

  public default void setPivotSpeed(double angle) {}

  public default void setPivotSpeedUp() {}

  public default void setPivotSpeedDown() {}

  public default double getIntakeAngle() {
    return 0.0;
  }

  public default double getIntakeSpeed() {
    return 0.0;
  }

  public default boolean isIntakeDeployed() {
    return false;
  }

  public default void periodic() {}
}
