package frc.robot.subsystems.adjustableHood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class PhysicalAdjustableHood {
    private final TalonFX hoodMotor1 = new TalonFX(AdjustableHoodConstants.HOOD_MOTOR_ID_1);
    private final TalonFX hoodMotor2 = new TalonFX(AdjustableHoodConstants.HOOD_MOTOR_ID_2);
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    public PhysicalAdjustableHood(){
        
    }
}
