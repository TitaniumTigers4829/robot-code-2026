// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {

  private SpindexerInterface spindexerInterface;
  private SpindexerInputsAutoLogged inputs = new SpindexerInputsAutoLogged();

  private static final LoggedTunableNumber spindexerS = new LoggedTunableNumber("Spindexer/SpindexerS");
  private static final LoggedTunableNumber spindexerV = new LoggedTunableNumber("Spindexer/SpindexerV");
  private static final LoggedTunableNumber spindexerA = new LoggedTunableNumber("Spindexer/SpindexerA");
  private static final LoggedTunableNumber spindexerP = new LoggedTunableNumber("Spindexer/SpindexerP");
  private static final LoggedTunableNumber spindexerI = new LoggedTunableNumber("Spindexer/SpindexerI");
  private static final LoggedTunableNumber spindexerD = new LoggedTunableNumber("Spindexer/SpindexerD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT -> {
        spindexerS.initDefault(SpindexerConstants.SPINDEXER_S);
        spindexerV.initDefault(SpindexerConstants.SPINDEXER_V);
        spindexerA.initDefault(SpindexerConstants.SPINDEXER_A);
        spindexerP.initDefault(SpindexerConstants.SPINDEXER_P);
        spindexerI.initDefault(SpindexerConstants.SPINDEXER_I);
        spindexerD.initDefault(SpindexerConstants.SPINDEXER_D);
      }
    }
  }

  /** Creates a new SpindexerSubsystem. */
  public SpindexerSubsystem(SpindexerInterface spindexerInterface) {
    this.spindexerInterface = spindexerInterface;
  }

  public double getVolts() {
    return spindexerInterface.getVolts();
  }

  public void setSpindexerSpeed(double targetRPM) {
    spindexerInterface.setSpindexerSpeed(targetRPM);
  }

  public double getSpindexerVelocity() {
    return inputs.spindexerVelocity;
  }

  public void setVolts(double volts) {
    spindexerInterface.setVolts(volts);
  }

  public void openLoop(double output) {
    spindexerInterface.openLoop(output);
  }

  public void setPercentOutput(double output) {
    spindexerInterface.setPercentOutput(output);
  }

  public void setSpeed(double speed) {
    spindexerInterface.setSpeed(speed);
  }

  @Override
  public void periodic() {
    spindexerInterface.updateInputs(inputs);
    Logger.processInputs("Spindexer/", inputs);

    // Update tunable numbers
    if (spindexerS.hasChanged(hashCode())
        || spindexerV.hasChanged(hashCode())
        || spindexerA.hasChanged(hashCode())) {
      spindexerInterface.setFF(spindexerS.get(), spindexerV.get(), spindexerA.get());
    }
    if (spindexerP.hasChanged(hashCode())
        || spindexerI.hasChanged(hashCode())
        || spindexerD.hasChanged(hashCode())) {
      spindexerInterface.setPID(spindexerP.get(), spindexerI.get(), spindexerD.get());
    }
  }
}
