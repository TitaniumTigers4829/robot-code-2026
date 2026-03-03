package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.HardwareConstants;

public class PhysicalIntake implements IntakeInterface {
  private TalonFX intakeMotorOuter = new TalonFX(IntakeConstants.INTAKE_MOTOR_1_ID);
  private TalonFX intakeMotorInside = new TalonFX(IntakeConstants.INTAKE_MOTOR_2_ID);
  private TalonFX intakePivotMotor1 =
      new TalonFX(IntakeConstants.PIVOT_MOTOR_1_ID, HardwareConstants.RIO_CAN_BUS_STRING);
  private TalonFX intakePivotMotor2 = new TalonFX(IntakeConstants.PIVOT_MOTOR_2_ID);

  private MotionMagicVoltage request = new MotionMagicVoltage(0.0);
  private MotorAlignmentValue pivotMotorAlignment = MotorAlignmentValue.Opposed;
  private TalonFXConfiguration intakeConfig;
  private TalonFXConfiguration pivotConfig;

  public StatusSignal<Angle> intakeAngle;
  public StatusSignal<AngularVelocity> intakePivotSpeed;

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
    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;

    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_DOWN_POSITION;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.MIN_ANGLE;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    intakeMotorOuter.getConfigurator().apply(intakeConfig);
    intakeMotorInside.getConfigurator().apply(intakeConfig);
    intakePivotMotor1.getConfigurator().apply(pivotConfig);
    intakePivotMotor2.getConfigurator().apply(pivotConfig);

    pivotConfig.MotionMagic.MotionMagicAcceleration = 4;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 10;

    intakeAngle = intakePivotMotor1.getPosition();
    intakePivotSpeed = intakePivotMotor1.getVelocity();

    intakePivotMotor1.setPosition(0.0);
    intakePivotMotor2.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(0.0, intakeAngle, intakePivotSpeed);
    ParentDevice.optimizeBusUtilizationForAll(
        intakeMotorOuter, intakeMotorInside, intakePivotMotor1, intakePivotMotor2);
  }

  public void updateInputs(IntakeInputs inputs) {
    // BaseStatusSignal.refreshAll(intakeAngle, intakePivotSpeed);
    intakeAngle.refresh();
    intakePivotSpeed.refresh();
    inputs.intakeAngle = intakeAngle.getValueAsDouble();
    inputs.intakePivotSpeed = intakePivotSpeed.getValueAsDouble();
    inputs.isIntakeDeployed = isIntakeDeployed();
  }

  public void setIntakeAngle(double angle) {
    intakePivotMotor1.setControl(request.withPosition(angle));
    intakePivotMotor2.setControl(
        new Follower(intakePivotMotor1.getDeviceID(), pivotMotorAlignment));
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

  public void setPivotSpeedUp() {
    intakePivotMotor1.setControl(request.withPosition(IntakeConstants.MAX_ANGLE));
    intakePivotMotor2.setControl(
        new Follower(intakePivotMotor2.getDeviceID(), pivotMotorAlignment));
  }

  public void setPivotSpeedDown() {
    intakePivotMotor1.setControl(request.withPosition(IntakeConstants.PIVOT_DOWN_POSITION));
    intakePivotMotor2.setControl(
        new Follower(intakePivotMotor2.getDeviceID(), pivotMotorAlignment));
  }

  // public double getIntakeAngle() {
  //   intakeAngle.refresh();
  //   return intakeAngle.getValueAsDouble();
  // }

  // public double getIntakeSpeed() {
  //   intakePivotSpeed.refresh();
  //   return intakePivotSpeed.getValueAsDouble();
  // }

  // public boolean isIntakeDeployed() {
  //   if (intakeAngle.getValueAsDouble()
  //           >= (IntakeConstants.PIVOT_DOWN_POSITION - IntakeConstants.ACCEPTABLE_RANGE)
  //       && intakeAngle.getValueAsDouble() <= IntakeConstants.PIVOT_DOWN_POSITION) {
  //     return true;
  //   } else {
  //     return false; // this code deport Andrita
  //   }
  // }
}
