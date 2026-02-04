package frc.robot.subsystems.adjustableHood;

import java.util.Set;

import org.littletonrobotics.junction.AutoLog;

public interface AdjustableHoodInterface {
    @AutoLog
    public static class AdjustableHoodInputs{
        public double hoodAngle = 0.0;
        public double hoodSpeed1 = 0.0;
        public double hoodSpeed2 = 0.0;
        
    }

    public default void updateInputs(AdjustableHoodInputs inputs){}

    public default void setHoodAngle(double angle){}

    public default void setSpeed(double speed){}

    public default double getHoodAngle(){
        return 0;
    }
}
