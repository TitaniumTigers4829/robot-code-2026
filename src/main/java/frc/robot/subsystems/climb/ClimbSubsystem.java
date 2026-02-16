// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  private ClimbInterface climbInterface;
  private ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

  private static final LoggedTunableNumber climbS = new LoggedTunableNumber("Climb/ClimbS");
  private static final LoggedTunableNumber climbV = new LoggedTunableNumber("Climb/ClimbV");
  private static final LoggedTunableNumber climbA = new LoggedTunableNumber("Climb/ClimbA");
  private static final LoggedTunableNumber climbP = new LoggedTunableNumber("Climb/ClimbP");
  private static final LoggedTunableNumber climbI = new LoggedTunableNumber("Climb/ClimbI");
  private static final LoggedTunableNumber climbD = new LoggedTunableNumber("Climb/ClimbD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT -> {
        climbS.initDefault(ClimbConstants.CLIMB_S);
        climbV.initDefault(ClimbConstants.CLIMB_V);
        climbA.initDefault(ClimbConstants.CLIMB_A);
        climbP.initDefault(ClimbConstants.CLIMB_P);
        climbI.initDefault(ClimbConstants.CLIMB_I);
        climbD.initDefault(ClimbConstants.CLIMB_D);
      }
    }
  }

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem(ClimbInterface climbInterface) {
    this.climbInterface = climbInterface;
  }


  public double getVolts() {
    return climbInterface.getVolts();
  }


  public double getClimbVelocity() {
    return inputs.climbVelocity;
  }

  public void setVolts(double volts) {
    climbInterface.setVolts(volts);
  }

  public void levelOne() {
    climbInterface.levelOne();
  }

  public void openLoop(double output) {
    climbInterface.openLoop(output);
  }

  public void setPercentOutput(double distance) {
    climbInterface.setPercentOutput(distance);
  }

  @Override
  public void periodic() {
    climbInterface.updateInputs(inputs);
    Logger.processInputs("Climb/", inputs);

    // Update tunable numbers
    if (climbS.hasChanged(hashCode())
        || climbV.hasChanged(hashCode())
        || climbA.hasChanged(hashCode())) {
      climbInterface.setFF(climbS.get(), climbV.get(), climbA.get());
    }
    if (climbP.hasChanged(hashCode())
        || climbI.hasChanged(hashCode())
        || climbD.hasChanged(hashCode())) {
      climbInterface.setPID(climbP.get(), climbI.get(), climbD.get());
    }
  }
}
