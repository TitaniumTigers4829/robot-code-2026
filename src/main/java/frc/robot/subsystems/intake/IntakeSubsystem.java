// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;

public class IntakeSubsystem extends SubsystemBase {

  private IntakeInterface intakeInterface;
  private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  private static final LoggedTunableNumber IntakeS = new LoggedTunableNumber("Intake/IntakeS");
  private static final LoggedTunableNumber IntakeV = new LoggedTunableNumber("Intake/IntakeV");
  private static final LoggedTunableNumber IntakeA = new LoggedTunableNumber("Intake/IntakeA");
  private static final LoggedTunableNumber IntakeP = new LoggedTunableNumber("Intake/IntakeP");
  private static final LoggedTunableNumber IntakeI = new LoggedTunableNumber("Intake/IntakeI");
  private static final LoggedTunableNumber IntakeD = new LoggedTunableNumber("Intake/IntakeD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT -> {
        IntakeS.initDefault(IntakeConstants.INTAKE_S);
        IntakeV.initDefault(IntakeConstants.INTAKE_V);
        IntakeA.initDefault(IntakeConstants.INTAKE_A);
        IntakeP.initDefault(IntakeConstants.INTAKE_P);
        IntakeI.initDefault(IntakeConstants.INTAKE_I);
        IntakeD.initDefault(IntakeConstants.INTAKE_D);
      }
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // default constructor
  }

  /** Creates a new IntakeSubsystem with hardware interface. */
  public IntakeSubsystem(IntakeInterface intakeInterface) {
    this.intakeInterface = intakeInterface;
  }

  public void setIntakeAngle(double angle) {
    this.intakeInterface.setIntakeAngle(angle);
  }

  public void intakeFuel(double speed) {
    this.intakeInterface.intakeFuel(speed);
  }

  public double getIntakeAngle() {
    return this.intakeInterface.getIntakeAngle();
  }

  public double getIntakeSpeed() {
    return this.intakeInterface.getIntakeSpeed();
  }

  public boolean isIntakeDeployed() {
    return this.intakeInterface.isIntakeDeployed();
  }
  public void updateInputs() {
    this.intakeInterface.updateInputs(inputs);
  }

  
  @Override
  public void periodic() {
    
    this.intakeInterface.periodic();
  }
}
