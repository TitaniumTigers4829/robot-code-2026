package frc.robot.subsystems.adjustableHood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdjustableHoodSubsystem extends SubsystemBase{
    private final AdjustableHoodInterface adjustableHoodInterface;
    private AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();

    public AdjustableHoodSubsystem(AdjustableHoodInterface adjustableHoodInterface) {
        this.adjustableHoodInterface = adjustableHoodInterface;
    }

    public void updateInputs(){
        adjustableHoodInterface.updateInputs(inputs);
    }

    public void setHoodAngle(double angle){
        adjustableHoodInterface.setHoodAngle(angle);
    }

    public void setSpeed(double speed){
        adjustableHoodInterface.setSpeed(speed);
    }

    public double getHoodAngle(){
        return adjustableHoodInterface.getHoodAngle();
    }
}
