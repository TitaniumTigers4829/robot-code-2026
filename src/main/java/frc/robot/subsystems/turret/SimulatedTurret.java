package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimulatedTurret implements TurretInterface {
  private final SingleJointedArmSim turretSim;
  private final PIDController pidController;
  private final SimpleMotorFeedforward feedforward;
  private double currentVolts;
  private double desiredAngle;

  public SimulatedTurret() {
    turretSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            TurretConstants.GEAR_RATIO,
            TurretConstants.TURRET_MOI,
            TurretConstants.TURRET_RADIUS,
            -Math.PI,
            Math.PI,
            false,
            0);
    pidController =
        new PIDController(
            TurretConstants.TURRET_P, TurretConstants.TURRET_I, TurretConstants.TURRET_D);
    feedforward =
        new SimpleMotorFeedforward(
            TurretConstants.TURRET_S, TurretConstants.TURRET_V, TurretConstants.TURRET_A);
    // currentVolts = 0.0;
    // desiredAngle = (Math.PI/2);
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
    turretSim.update(0.02);
    inputs.turretAngle = turretSim.getAngleRads();
    inputs.turretMotorAppliedVoltage = currentVolts;
    inputs.turretDesiredAngle = desiredAngle;
  }

  @Override
  public void setTurretAngle(double angle) {
    // desiredAngle = angle;
    double pidOutput = pidController.calculate(getTurretAngle(), angle);
    double feedforwardOutput =
        feedforward.calculate(getTurretAngle(), turretSim.getVelocityRadPerSec());
    currentVolts = pidOutput + feedforwardOutput;
    turretSim.setInputVoltage(currentVolts);
  }

  @Override
  public double getTurretAngle(){
    return turretSim.getAngleRads();
  }

  @Override
  public void setVolts(double volts) {
    currentVolts = volts;
    turretSim.setInputVoltage(currentVolts);
  }

  @Override
  public double getVolts() {
    return currentVolts;
  }
}
