package frc.robot.subsystems.adjustableHood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

public class PhysicalAdjustableHood implements AdjustableHoodInterface {
  private final TalonFX hoodMotor =
      new TalonFX(AdjustableHoodConstants.HOOD_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
  private final CANcoder hoodEncoder =
      new CANcoder(AdjustableHoodConstants.CANCODER_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);

  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
  private final SingleLinearInterpolator adjustableHoodLookupValues =
      new SingleLinearInterpolator(AdjustableHoodConstants.hoodLookUpTable);

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut current = new DutyCycleOut(0.0);
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

    hoodConfig.ClosedLoopGeneral.ContinuousWrap = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.Feedback.SensorToMechanismRatio = AdjustableHoodConstants.GEAR_RATIO;

    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1.2;
    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.CurrentLimits.StatorCurrentLimit = 40;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodEncoderConfig.MagnetSensor.MagnetOffset = AdjustableHoodConstants.HOOD_ZERO_ANGLE;
    hoodEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    hoodConfig.MotionMagic.MotionMagicAcceleration = 10;
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 4;

    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

    hoodAngle = hoodEncoder.getPosition();

    // hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
    hoodAngle.setUpdateFrequency(250.0);
    hoodEncoder.getAbsolutePosition().setUpdateFrequency(250);
    ParentDevice.optimizeBusUtilizationForAll(hoodMotor, hoodEncoder);
  }

  @Override
  public void updateInputs(AdjustableHoodInputs inputs) {
    hoodAngle.refresh();
    hoodEncoder.getAbsolutePosition().refresh();
    inputs.hoodAbsPos = hoodEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.hoodAngle = hoodAngle.getValueAsDouble();
    inputs.curentLookupTable = lookupTableStuff;
    inputs.distanceGiven = distanceGiven;
  }

  @Override
  public double getHoodAngle() {
    hoodAngle.refresh();
    return hoodAngle.getValueAsDouble();
  }

  @Override
  public void setHoodAngle(double distance) {
    lookupTableStuff = adjustableHoodLookupValues.getLookupValue(distance);
    distanceGiven = distance;
    hoodMotor.setControl(positionRequest.withPosition(lookupTableStuff));
  }

  @Override
  public void setAngleWithoutDist(double rots) {
    hoodMotor.setControl(positionRequest.withPosition(rots));
  }

  @Override
  public void setSpeed(double speed) {
    hoodMotor.setControl(current.withOutput(speed));
  }

  @Override
  public void rezeroHood() {
    hoodEncoder.setPosition(0.0);
  }
}
