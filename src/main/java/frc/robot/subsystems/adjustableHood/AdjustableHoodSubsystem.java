package frc.robot.subsystems.adjustableHood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdjustableHoodSubsystem extends SubsystemBase{
    private final AdjustableHoodInterface adjustableHoodInterface;
    private AdjustableHoodInputsAutoLogged inputs = new AdjustableHoodInputsAutoLogged();

    public AdjustableHoodSubsystem(AdjustableHoodInterface adjustableHoodInterface) {
        this.adjustableHoodInterface = adjustableHoodInterface;
    }


}
