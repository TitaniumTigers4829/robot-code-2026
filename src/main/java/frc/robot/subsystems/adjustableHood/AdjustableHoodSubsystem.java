package frc.robot.subsystems.adjustableHood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AdjustableHoodSubsystem extends SubsystemBase {
  private final AdjustableHoodInterface adjustableHoodInterface;
  private AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();

  public AdjustableHoodSubsystem(AdjustableHoodInterface adjustableHoodInterface) {
    this.adjustableHoodInterface = adjustableHoodInterface;
  }

  public void setHoodAngle(double angle) {
    adjustableHoodInterface.setHoodAngle(angle);
  }

  public void setSpeed(double speed) {
    adjustableHoodInterface.setSpeed(speed);
  }

  public void setAngleWithoutDist(double rots) {
    adjustableHoodInterface.setAngleWithoutDist(rots);
  }

  public double getHoodAngle() {
    return adjustableHoodInterface.getHoodAngle();
  }

  public void rezeroHood() {
    adjustableHoodInterface.rezeroHood();
  }

  public void resetHoodPID() {
    adjustableHoodInterface.resetHoodPID();
  }

  @Override
  public void periodic() {
    adjustableHoodInterface.updateInputs(inputs);
    Logger.recordOutput("hoodAngle", inputs.hoodAngle);
    Logger.recordOutput("hoodAbsPos", inputs.hoodAbsPos);
    Logger.recordOutput("desired angle", inputs.desiredAngle);
    // if (getHoodAngle() < 0 && inputs.desiredAngle < 0.01) {
    //   setSpeed(0);
    // }
  }
}
