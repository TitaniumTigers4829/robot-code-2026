// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.logging.LoggedTunableNumber;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;
import frc.robot.subsystems.intake.IntakeConstants;

/** Add your docs here. */
public class PhysicalShooter implements ShooterInterface {

  private boolean isUpToSpeed = false;
  private boolean isAimingProperly = false;

  LoggedTunableNumber flywheelRPS = new LoggedTunableNumber("Shooter/RPS", 0.0);

  private final TalonFX leaderFlywheelMotor =
      new TalonFX(
          ShooterConstants.LEADER_FLYWHEEL_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
  private final TalonFX followerFlywheelMotor =
      new TalonFX(
          ShooterConstants.FOLLOWER_FLYWHEEL_MOTOR_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);
  private final TalonFX kickerMotor = new TalonFX(ShooterConstants.KICKER_MOTOR_ID);
  private final TalonFX spindexerMotor = new TalonFX(ShooterConstants.SPINDEXER_MOTOR_ID);
  private final TalonFX insideRollerMotor =
      new TalonFX(IntakeConstants.INTAKE_MOTOR_2_ID, HardwareConstants.RIO_CAN_BUS_STRING);
  MotorAlignmentValue motorAlignment = MotorAlignmentValue.Opposed;

  private final SingleLinearInterpolator flywheelRPMLookupValues;

  private final VelocityTorqueCurrentFOC rpsRequest = new VelocityTorqueCurrentFOC(0.0);
  private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);

  // private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new
  // MotionMagicTorqueCurrentFOC(0.0);
  // private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  private final TalonFXConfiguration leaderFlywheelConfig = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> currentRPS;

  // private final TalonFXConfiguration followerFlywheelConfig = new TalonFXConfiguration();

  // TODO: Add configs, and make slav
  public PhysicalShooter() {

    leaderFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
    leaderFlywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_P;
    leaderFlywheelConfig.Slot0.kI = ShooterConstants.FLYWHEEL_I;
    leaderFlywheelConfig.Slot0.kD = ShooterConstants.FLYWHEEL_D;
    leaderFlywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_S;
    leaderFlywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_V;
    leaderFlywheelConfig.Slot0.kA = ShooterConstants.FLYWHEEL_A;
    // TODO: drop down
    leaderFlywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
    
    // leaderFlywheelConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leaderFlywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    leaderFlywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

    leaderFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // leaderFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // TODO: timeouts
    // TODO: unscuff
    leaderFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    followerFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    // kickerMotor.getConfigurator().apply(leaderFlywheelConfig);
    // leaderFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // spindexerMotor.inve
    TalonFXConfiguration spindexerAndKickerConfig = new TalonFXConfiguration();
    // TODO: tune
    spindexerAndKickerConfig.CurrentLimits.StatorCurrentLimit = 60;
    spindexerAndKickerConfig.CurrentLimits.SupplyCurrentLimit = 30;
    spindexerAndKickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerAndKickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    spindexerAndKickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kickerMotor.getConfigurator().apply(spindexerAndKickerConfig);
    spindexerAndKickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    spindexerMotor.getConfigurator().apply(spindexerAndKickerConfig);
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));

    flywheelRPMLookupValues =
        new SingleLinearInterpolator(ShooterConstants.DISTANCE_TO_FLYWHEEL_RPM);

    currentRPS = leaderFlywheelMotor.getVelocity();

    currentRPS.setUpdateFrequency(100);
    ParentDevice.optimizeBusUtilizationForAll(leaderFlywheelMotor);
  }

  public void updateInputs(ShooterInputs inputs) {
    BaseStatusSignal.refreshAll(currentRPS);

    inputs.flywheelRPS = currentRPS.getValueAsDouble();
  }

  public double getFlywheelRPS() {
    currentRPS.refresh();
    return currentRPS.getValueAsDouble();
  }

  // test
  public void setPercentOutput(double distance) {
    double desiredSpeed = flywheelRPMLookupValues.getLookupValue(distance);
    // double desiredSpeed = flywheelRPS.get();
    leaderFlywheelMotor.setControl(rpsRequest.withVelocity(desiredSpeed));
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));
    this.isUpToSpeed =
        Math.abs(desiredSpeed - currentRPS.refresh().getValueAsDouble())
            < ShooterConstants.FLYWHEEL_ERROR_TOLERANCE;
    // SmartDashboard.putNumber("desiredRPS", desiredSpeed);
    // SmartDashboard.putNumber("currentRPS", currentRPS.refresh().getValueAsDouble());
    setKickerSpeed(ShooterConstants.KICKER_PERCENT_OUTPUT);

    SmartDashboard.putBoolean("ready to shoot", isUpToSpeed() && this.isAimingProperly);

    if (isUpToSpeed() && this.isAimingProperly) {
      setSpindexerSpeed(ShooterConstants.SPINDEXER_SHOOT_SPEED);
      // setKickerSpeed(ShooterConstants.KICKER_PERCENT_OUTPUT);
      // setRollerSpeed(0.2);
    } else {
      setSpindexerSpeed(0.0);
      // setKickerSpeed(0.0);
      // setRollerSpeed(0.0);
    }
  }

  public boolean isUpToSpeed() {
    return this.isUpToSpeed;
  }

  public void passFuel() {
    leaderFlywheelMotor.setControl(rpsRequest.withVelocity(50));
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));
    this.isUpToSpeed =
        Math.abs(50 - currentRPS.refresh().getValueAsDouble())
            < ShooterConstants.FLYWHEEL_ERROR_TOLERANCE;
    // SmartDashboard.putNumber("desiredRPS", 60);
    // SmartDashboard.putNumber("currentRPS", currentRPS.refresh().getValueAsDouble());
    if (isUpToSpeed()) {
      setSpindexerSpeed(ShooterConstants.SPINDEXER_SHOOT_SPEED);
      setKickerSpeed(ShooterConstants.KICKER_PERCENT_OUTPUT);
      // setRollerSpeed(0.2);

    } else {
      setSpindexerSpeed(0.0);
      setKickerSpeed(0.0);
      // setRollerSpeed(0.0);
    }
  }

  public void setSpeed(double rps) {
    leaderFlywheelMotor.setControl(rpsRequest.withVelocity(rps));
    followerFlywheelMotor.setControl(
        new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));
  }

  public void setPercentOuput2(double speed) {
    leaderFlywheelMotor.set(speed);
    followerFlywheelMotor.set(-1 * speed);
  }

  public void stopShoot() {
    leaderFlywheelMotor.set(0);
    followerFlywheelMotor.set(0);
    kickerMotor.set(0);
    spindexerMotor.set(0);
  }

  public void setSpindexerSpeed(double speed) {
    spindexerMotor.set(speed);
  }

  public void setKickerSpeed(double speed) {
    kickerMotor.set(speed);
  }

  public void setRollerSpeed(double speed) {
    insideRollerMotor.set(speed);
  }

  public void setIsAimingProperly(boolean isAimingProperly) {
    this.isAimingProperly = isAimingProperly;
  }
}
