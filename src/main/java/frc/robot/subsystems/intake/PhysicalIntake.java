package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PhysicalIntake implements IntakeInterface {
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    private TalonFX intakePivotMotor1 = new TalonFX(IntakeConstants.PIVOT_MOTOR_1_ID);
    private TalonFX intakePivotMotor2 = new TalonFX(IntakeConstants.PIVOT_MOTOR_2_ID);
    private CANcoder intakeCanCoder1 = new CANcoder(IntakeConstants.INTAKE_CAN_CODER_1_ID);
    private CANcoder intakeCanCoder2 = new CANcoder(IntakeConstants.INTAKE_CAN_CODER_2_ID);

    private MotorAlignmentValue pivotMotorAlignment = MotorAlignmentValue.Opposed;
    private TalonFXConfiguration intakeConfig;
    private TalonFXConfiguration pivotConfig;

    public PhysicalIntake() {
        intakeConfig = new TalonFXConfiguration();
        pivotConfig = new TalonFXConfiguration();
        
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.Slot0.kP = IntakeConstants.INTAKE_P;
        intakeConfig.Slot0.kI = IntakeConstants.INTAKE_I;
        intakeConfig.Slot0.kD = IntakeConstants.INTAKE_D;
        intakeConfig.Slot0.kS = IntakeConstants.INTAKE_S;
        intakeConfig.Slot0.kV = IntakeConstants.INTAKE_V;
        intakeConfig.Slot0.kA = IntakeConstants.INTAKE_A;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.Slot0.kP = IntakeConstants.PIVOT_P;
        pivotConfig.Slot0.kI = IntakeConstants.PIVOT_I;
        pivotConfig.Slot0.kD = IntakeConstants.PIVOT_D;
        pivotConfig.Slot0.kS = IntakeConstants.PIVOT_S;
        pivotConfig.Slot0.kV = IntakeConstants.PIVOT_V;
        pivotConfig.Slot0.kA = IntakeConstants.PIVOT_A;
        pivotConfig.Slot0.kG = IntakeConstants.PIVOT_G;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.MAX_ANGLE;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.MIN_ANGLE;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        intakeMotor.getConfigurator().apply(intakeConfig);
        intakePivotMotor1.getConfigurator().apply(pivotConfig);
        intakePivotMotor2.getConfigurator().apply(pivotConfig);
    }

    public void updateInputs(IntakeInputs inputs) {}

    public void setIntakeAngle(double angle) {}

    public void setIntakeSpeed(double speed) {}

    public double getIntakeAngle() { 
        return 0.0; 
    }

    public double getIntakeSpeed() { 
        return 0.0; 
    }

    public boolean isIntakeDeployed() { 
        return false; 
    }
}