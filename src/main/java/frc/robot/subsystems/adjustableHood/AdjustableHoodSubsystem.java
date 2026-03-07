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

  @Override
  public void periodic() {
    adjustableHoodInterface.updateInputs(inputs);
    Logger.recordOutput("hoodAngle", getHoodAngle());
    Logger.recordOutput("hoodAbsPos", inputs.hoodAbsPos);
    Logger.recordOutput("desired angle", inputs.desiredAngle);
    Logger.recordOutput("lookup table", inputs.curentLookupTable);
    Logger.recordOutput("distance given", inputs.distanceGiven);
  }
}
