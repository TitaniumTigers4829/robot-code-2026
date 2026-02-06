package frc.robot.subsystems.adjustableHood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

public class PhysicalAdjustableHood implements AdjustableHoodInterface{
    private final TalonFX hoodMotor = new TalonFX(AdjustableHoodConstants.HOOD_MOTOR_ID);
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    private final SingleLinearInterpolator adjustableHoodLookupValues = new SingleLinearInterpolator(AdjustableHoodConstants.hoodLookUpTable);

    private final MotionMagicTorqueCurrentFOC mmmmmm = new MotionMagicTorqueCurrentFOC(0.0);
    private final TorqueCurrentFOC current = new TorqueCurrentFOC(0.0);
    public StatusSignal<Angle> hoodAngle;

    public PhysicalAdjustableHood(){
        hoodMotor.getConfigurator().apply(hoodConfig);

        hoodAngle.setUpdateFrequency(100.0);
        ParentDevice.optimizeBusUtilizationForAll(hoodMotor);
    }

    public void updateInputs(AdjustableHoodInputs inputs){
        BaseStatusSignal.refreshAll(
            hoodAngle
        );

        inputs.hoodAngle = hoodAngle.getValueAsDouble();
    }

    public double getHoodAngle(){
        hoodAngle.refresh();
        return hoodAngle.getValueAsDouble();
    }

    public void periodic(){
        SmartDashboard.putNumber("hoodAngle", getHoodAngle());
    }
}
