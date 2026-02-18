// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;
import frc.robot.extras.logging.Tracer;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

  private TurretInterface turretInterface;
  private TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();

  private static final LoggedTunableNumber turretS = new LoggedTunableNumber("Turret/TurretS");
  private static final LoggedTunableNumber turretV = new LoggedTunableNumber("Turret/TurretV");
  private static final LoggedTunableNumber turretA = new LoggedTunableNumber("Turret/TurretA");
  private static final LoggedTunableNumber turretP = new LoggedTunableNumber("Turret/TurretP");
  private static final LoggedTunableNumber turretI = new LoggedTunableNumber("Turret/TurretI");
  private static final LoggedTunableNumber turretD = new LoggedTunableNumber("Turret/TurretD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT -> {
        turretP.initDefault(TurretConstants.TURRET_P);
        turretI.initDefault(TurretConstants.TURRET_I);
        turretD.initDefault(TurretConstants.TURRET_D);
      }
    }
  }

  /** Creates a new turretSubsystem. */
  public TurretSubsystem(TurretInterface turretInterface) {
    this.turretInterface = turretInterface;
  }

  public double getTurretAngle() {
    return turretInterface.getTurretAngle();
  }

  public double getVolts() {
    return turretInterface.getVolts();
  }

  // In rotations
  public void setTurretAngle(double targetAngle) {
    turretInterface.setTurretAngle(targetAngle);
  }

  public void setVolts(double volts) {
    turretInterface.setVolts(volts);
  }

  public void openLoop(double output) {
    turretInterface.openLoop(output);
  }

  public void setPercentOutput(double output) {
    turretInterface.setPercentOutput(output);
  }

  public void setSpeed(double speed) {
    turretInterface.setSpeed(speed);
  }

  public boolean iAtSetpointAngle(double targetAngle) {
    return Math.abs(targetAngle - inputs.turretAngle) < TurretConstants.TURRET_ERROR_TOLERANCE;
  }

  @Override
  public void periodic() {
    turretInterface.updateInputs(inputs);
    Logger.processInputs("turret/", inputs);
    Tracer.traceFunc("Turret/", () -> turretInterface.updateInputs(inputs));

    // Update tunable numbers
    if (turretP.hasChanged(hashCode())
        || turretI.hasChanged(hashCode())
        || turretD.hasChanged(hashCode())) {
      turretInterface.setPID(turretP.get(), turretI.get(), turretD.get());
    }

    if (turretS.hasChanged(hashCode())
        || turretV.hasChanged(hashCode())
        || turretA.hasChanged(hashCode())) {
      turretInterface.setFF(turretS.get(), turretV.get(), turretA.get());
    }

    // SmartDashboard.putNumber("turret angle", inputs.turretAngle);
    // SmartDashboard.putNumber("error")
  }
}
