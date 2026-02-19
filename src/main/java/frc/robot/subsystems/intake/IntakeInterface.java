package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeInterface {
  @AutoLog
  public static class IntakeInputs {
    public boolean isIntakeDeployed = false;
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorCurrent = 0.0;
    public double intakeAngle = 0.0;
    public double intakeSpeed = 0.0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setIntakeAngle(double angle) {}

  public default void intakeFuel(double speed) {}

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
