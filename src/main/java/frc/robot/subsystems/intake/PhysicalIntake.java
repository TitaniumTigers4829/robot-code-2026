package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HardwareConstants;

public class PhysicalIntake implements IntakeInterface {
  private TalonFX intakeMotorOuter = new TalonFX(IntakeConstants.INTAKE_MOTOR_1_ID);
  private TalonFX intakeMotorInside = new TalonFX(IntakeConstants.INTAKE_MOTOR_2_ID);
  private TalonFX intakePivotMotorRight =
      new TalonFX(IntakeConstants.PIVOT_MOTOR_RIGHT_ID, HardwareConstants.RIO_CAN_BUS_STRING);
  private TalonFX intakePivotMotorLeft = new TalonFX(IntakeConstants.PIVOT_MOTOR_LEFT_ID);

  private final CANcoder pivotEncoder =
      new CANcoder(IntakeConstants.CANCODER_ID, HardwareConstants.RIO_CAN_BUS_STRING);

  private MotionMagicVoltage request = new MotionMagicVoltage(0.0);
  private MotorAlignmentValue pivotMotorAlignment = MotorAlignmentValue.Opposed;
  private TalonFXConfiguration intakeConfig;
  private TalonFXConfiguration pivotConfig;
  private CANcoderConfiguration encoderConfig;

  public StatusSignal<Angle> intakeAngle;
  public StatusSignal<AngularVelocity> intakePivotSpeed;

  public PhysicalIntake() {
    intakeConfig = new TalonFXConfiguration();
    pivotConfig = new TalonFXConfiguration();
    encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.ZERO_ANGLE;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotEncoder.getConfigurator().apply(encoderConfig, HardwareConstants.TIMEOUT_SECONDS);

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
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
    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_DOWN_POSITION;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.MIN_ANGLE;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    intakeMotorOuter.getConfigurator().apply(intakeConfig);
    intakeMotorInside.getConfigurator().apply(intakeConfig);
    intakePivotMotorRight.getConfigurator().apply(pivotConfig);
    intakePivotMotorLeft.getConfigurator().apply(pivotConfig);

    intakeAngle = pivotEncoder.getPosition();
    intakePivotSpeed = intakePivotMotorRight.getVelocity();

    intakePivotMotorRight.setPosition(0.0);
    intakePivotMotorLeft.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, intakeAngle, intakePivotSpeed);
    ParentDevice.optimizeBusUtilizationForAll(
        intakeMotorOuter, intakeMotorInside, intakePivotMotorRight, intakePivotMotorLeft);
  }

  public void updateInputs(IntakeInputs inputs) {
    intakeAngle.refresh();
    intakePivotSpeed.refresh();

    inputs.intakeAngle = intakeAngle.getValueAsDouble();
    inputs.intakePivotSpeed = intakePivotSpeed.getValueAsDouble();
    inputs.isIntakeDeployed = isIntakeDeployed();
    SmartDashboard.putNumber("intake angle", inputs.intakeAngle);
    // System.out.println("WORKING");
  }

  public void setIntakeAngle(double angle) {
    intakePivotMotorRight.setControl(request.withPosition(angle));
    intakePivotMotorLeft.setControl(
        new Follower(intakePivotMotorRight.getDeviceID(), pivotMotorAlignment));
  }

  public void intakeFuel() {
    intakeMotorOuter.set(IntakeConstants.INTAKE_SPEED_OUTER);
    intakeMotorInside.set(IntakeConstants.INTAKE_SPEED_INNER);
  }

  public void outakeFuel() {
    intakeMotorOuter.set(-IntakeConstants.INTAKE_SPEED_OUTER);
    intakeMotorInside.set(-IntakeConstants.INTAKE_SPEED_INNER);
  }

  public void setSpeed() {
    intakeMotorOuter.set(0);
    intakeMotorInside.set(0);
  }

  public void setPivotSpeed(double speed) {
    intakePivotMotorRight.set(speed);
    intakePivotMotorLeft.setControl(
        new Follower(intakePivotMotorLeft.getDeviceID(), pivotMotorAlignment));
  }

  public void setAngle(double angle) {
    intakePivotMotorRight.setControl(request.withPosition(angle));
    intakePivotMotorLeft.setControl(
        new Follower(intakePivotMotorLeft.getDeviceID(), pivotMotorAlignment));
  }

  public void setPivotSpeedUp() {
    intakePivotMotorRight.setControl(request.withPosition(IntakeConstants.MIN_ANGLE));
    intakePivotMotorLeft.setControl(
        new Follower(intakePivotMotorLeft.getDeviceID(), pivotMotorAlignment));
  }

  public void setPivotSpeedDown() {
    intakePivotMotorRight.setControl(request.withPosition(IntakeConstants.PIVOT_DOWN_POSITION));
    intakePivotMotorLeft.setControl(
        new Follower(intakePivotMotorLeft.getDeviceID(), pivotMotorAlignment));
  }

  public double getIntakeAngle() {
    intakeAngle.refresh();
    return intakeAngle.getValueAsDouble();
  }

  public double getIntakeSpeed() {
    intakePivotSpeed.refresh();
    return intakePivotSpeed.getValueAsDouble();
  }

  public boolean isIntakeDeployed() {
    if (intakeAngle.getValueAsDouble()
            >= (IntakeConstants.PIVOT_DOWN_POSITION - IntakeConstants.ACCEPTABLE_RANGE)
        && intakeAngle.getValueAsDouble() <= IntakeConstants.PIVOT_DOWN_POSITION) {
      return true;
    } else {
      return false; // this code deport Andrita
    }
  }
}
