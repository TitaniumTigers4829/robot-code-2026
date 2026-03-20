package frc.robot.subsystems.adjustableHood;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

public class PhysicalAdjustableHood implements AdjustableHoodInterface {
  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;

  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();
  private final SingleLinearInterpolator adjustableHoodLookupValues =
      new SingleLinearInterpolator(AdjustableHoodConstants.hoodLookUpTable);

  // private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.0);
  private final DutyCycleOut current = new DutyCycleOut(0.0);
  public StatusSignal<Angle> hoodAngle;
  private StatusSignal<AngularVelocity> hoodVelocity;
  public double desiredAngle;
  public double lookupTableStuff = 0.0;
  public double distanceGiven = 0.0;
  private final PIDController hoodController =
      new PIDController(0.15, 0, 0.004); // 0.224829    0.004829
  private final SimpleMotorFeedforward ffHoodController = new SimpleMotorFeedforward(0.035, 0.0);

  public PhysicalAdjustableHood() {

    hoodMotor =
        new TalonFX(
            AdjustableHoodConstants.HOOD_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    hoodEncoder =
        new CANcoder(
            AdjustableHoodConstants.CANCODER_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);

    // hoodConfig.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    // TODO: try SyncCANCoder
    hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    hoodConfig.Slot0.kP = AdjustableHoodConstants.HOOD_P;
    hoodConfig.Slot0.kI = AdjustableHoodConstants.HOOD_I;
    hoodConfig.Slot0.kD = AdjustableHoodConstants.HOOD_D;
    hoodConfig.Slot0.kS = AdjustableHoodConstants.HOOD_S;
    hoodConfig.Slot0.kV = AdjustableHoodConstants.HOOD_V;
    hoodConfig.Slot0.kA = AdjustableHoodConstants.HOOD_A;

    hoodConfig.ClosedLoopGeneral.ContinuousWrap = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.1;
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.CurrentLimits.StatorCurrentLimit = 40;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // hoodEncoderConfig.MagnetSensor.MagnetOffset = -AdjustableHoodConstants.HOOD_ZERO_ANGLE;
    // hoodEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    // hoodEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    // hoodEncoderConfig.MagnetSensor.

    // hoodConfig.MotionMagic.MotionMagicAcceleration = 10;
    // hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 4;

    hoodMotor.getConfigurator().apply(hoodConfig, 0.02);
    // hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

    hoodMotor.setPosition(0);

    hoodAngle = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();

    // hoodEncoder.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());
    // TODO: this was 50hz, can go up to 1000hz
    hoodAngle.setUpdateFrequency(250.0);
    hoodVelocity.setUpdateFrequency(250);
    // hoodEncoder.getAbsolutePosition().setUpdateFrequency(50);
    ParentDevice.optimizeBusUtilizationForAll(hoodMotor, hoodEncoder);

    desiredAngle = 0;
  }

  @Override
  public void updateInputs(AdjustableHoodInputs inputs) {
    hoodAngle.refresh();
    hoodVelocity.refresh();
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
    // lookupTableStuff = adjustableHoodLookupValues.getLookupValue(distance);
    // distanceGiven = distance;
    // hoodMotor.setControl(positionRequest.withPosition(lookupTableStuff));
  }

  @Override
  public void setAngleWithoutDist(double rots) {
    // hoodMotor.setControl(positionRequest.withPosition(rots));
    hoodMotor.set(
        hoodController.calculate(getHoodAngle(), rots)
            + ffHoodController.calculate(hoodVelocity.refresh().getValueAsDouble()));

    // SmartDashboard.putNumber("pos req", positionRequest.Position);
    // SmartDashboard.putNumber("pos req 2", hoodMotor.getPosition().refresh().getValueAsDouble());
  }

  @Override
  public void setSpeed(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  public void rezeroHood() {
    hoodEncoder.setPosition(0.0);
    hoodMotor.setPosition(0);
  }

  @Override
  public void resetHoodPID() {
    hoodController.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hood angle", getHoodAngle());
  }
}
