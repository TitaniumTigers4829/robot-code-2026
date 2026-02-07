package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.simController.ClosedLoop;
import frc.robot.sim.simController.UnitSafeControl.AngularPIDFeedback;
import frc.robot.sim.simController.UnitSafeControl.AngularVelocityPIDFeedback;
import frc.robot.sim.simController.UnitSafeControl.FlywheelFeedforward;
import frc.robot.sim.simController.UnitSafeMotorController;
import frc.robot.sim.simMechanism.simSwerve.SimSwerve;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;

/** Wrapper class around {@link SwerveModuleSimulation} */
public class SimulatedModule implements ModuleInterface {
  private final UnitSafeMotorController driveMotor;
  private final UnitSafeMotorController steerMotor;
  private final ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> driveLoop;
  private final ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> steerLoop;

  public SimulatedModule(int moduleId, SimSwerve simSwerve) {
    driveMotor = new UnitSafeMotorController();
    steerMotor = new UnitSafeMotorController();

    driveLoop =
        ClosedLoop.forVoltageAngularVelocity(
            new AngularVelocityPIDFeedback<VoltageUnit>(
                Volts.per(RotationsPerSecond).ofNative(.20),
                Volts.per(RotationsPerSecondPerSecond).ofNative(0)),
            new FlywheelFeedforward<VoltageUnit>(
                Volts.of(.5),
                Volts.per(RotationsPerSecond).ofNative(1.0),
                Volts.per(RotationsPerSecondPerSecond).ofNative(0.0)));

    steerLoop =
        ClosedLoop.forVoltageAngle(
            new AngularPIDFeedback<VoltageUnit>(
                    Volts.per(Rotations).ofNative(18), Volts.per(RotationsPerSecond).ofNative(0))
                .withContinuousAngularInput(),
            new FlywheelFeedforward<VoltageUnit>(Volts.of(0)),
            true);

    steerMotor.configSensorToMechanismRatio(ModuleConstants.TURN_GEAR_RATIO);

    simSwerve.withSetModuleControllers(moduleId, driveMotor, steerMotor);
  }

  // TODO: maybe add supply?
  @Override
  public void updateInputs(ModuleInputs inputs) {
    // TODO: add drive accel
    inputs.drivePosition = -driveMotor.position().in(Rotations);
    inputs.driveVelocity = driveMotor.velocity().in(RotationsPerSecond);
    inputs.driveAppliedVolts = driveMotor.voltage().in(Volts);
    inputs.driveCurrentAmps = driveMotor.current().in(Amps);

    inputs.turnAbsolutePosition = steerMotor.position().in(Rotations);
    inputs.turnVelocity = steerMotor.velocity().in(RotationsPerSecond);

    inputs.turnAppliedVolts = steerMotor.voltage().in(Volts);
    inputs.turnCurrentAmps = steerMotor.current().in(Amps);

    inputs.isDriveConnected = true;
    inputs.isEncoderConnected = true;
    inputs.isTurnConnected = true;
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    volts = driveMotor.voltage();
  }

  @Override
  public void setTurnVoltage(Voltage volts) {
    volts = steerMotor.voltage();
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        desiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    driveMotor.controlVoltage(driveLoop, RotationsPerSecond.of(desiredDriveRPS));
    steerMotor.controlVoltage(steerLoop, desiredState.angle.getMeasure());
  }

  @Override
  public double getTurnRotations() {
    return steerMotor.position().in(Rotations);
  }

  @Override
  public void stopModule() {
    setDriveVoltage(Volts.zero());
    setTurnVoltage(Volts.zero());
  }
}
