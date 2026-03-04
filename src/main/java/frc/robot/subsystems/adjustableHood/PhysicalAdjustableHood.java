package frc.robot.subsystems.adjustableHood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

public class PhysicalAdjustableHood implements AdjustableHoodInterface {
  private final TalonFX hoodMotor = new TalonFX(AdjustableHoodConstants.HOOD_MOTOR_ID);
  private final CANcoder hoodEncoder = new CANcoder(AdjustableHoodConstants.CANCODER_ID);

  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
  private final SingleLinearInterpolator adjustableHoodLookupValues =
      new SingleLinearInterpolator(AdjustableHoodConstants.hoodLookUpTable);

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final TorqueCurrentFOC current = new TorqueCurrentFOC(0.0);
  public StatusSignal<Angle> hoodAngle;
  public double desiredAngle;
  public double lookupTableStuff = 0.0;
  public double distanceGiven = 0.0;

  public PhysicalAdjustableHood() {

    hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    hoodConfig.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();

    hoodConfig.Slot0.kP = AdjustableHoodConstants.HOOD_P;
    hoodConfig.Slot0.kI = AdjustableHoodConstants.HOOD_I;
    hoodConfig.Slot0.kD = AdjustableHoodConstants.HOOD_D;
    hoodConfig.Slot0.kS = AdjustableHoodConstants.HOOD_S;
    hoodConfig.Slot0.kV = AdjustableHoodConstants.HOOD_V;
    hoodConfig.Slot0.kA = AdjustableHoodConstants.HOOD_A;
    // hoodConfig.Slot0.kG = AdjustableHoodConstants.HOOD_G;

    hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    hoodConfig.ClosedLoopGeneral.ContinuousWrap = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.5;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.CurrentLimits.StatorCurrentLimit = 40;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodEncoderConfig.MagnetSensor.MagnetOffset = AdjustableHoodConstants.HOOD_ZERO_ANGLE;
    hoodEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    hoodConfig.MotionMagic.MotionMagicAcceleration = 1;
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 2;

    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

    hoodAngle = hoodEncoder.getAbsolutePosition();

    hoodEncoder.setPosition(0.0);

    hoodAngle.setUpdateFrequency(100.0);
    ParentDevice.optimizeBusUtilizationForAll(hoodMotor, hoodEncoder);
  }

  public void updateInputs(AdjustableHoodInputs inputs) {
    hoodAngle.refresh();
    inputs.hoodAngle = hoodAngle.getValueAsDouble();
    inputs.curentLookupTable = lookupTableStuff;
    inputs.distanceGiven = distanceGiven;
  }

  public double getHoodAngle() {
    hoodAngle.refresh();
    return hoodAngle.getValueAsDouble();
  }

  public void setHoodAngle(double distance) {
    lookupTableStuff = adjustableHoodLookupValues.getLookupValue(distance);
    distanceGiven = distance;
    hoodMotor.setControl(positionRequest.withPosition(lookupTableStuff));
  }

  public void setSpeed(double speed) {
    hoodMotor.set(speed);
  }

  public void periodic() {
    // SmartDashboard.putNumber("hoodAngle", getHoodAngle());
    // SmartDashboard.putNumber("desiredAngle", desiredAngle.getValueAsDouble());
  }
}
