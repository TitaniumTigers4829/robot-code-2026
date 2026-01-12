package frc.robot.sim.simController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.extras.util.ProceduralStructGenerator;
import frc.robot.sim.simMechanism.SimMechanism.MechanismState;

/**
 * An interface to represent a motor controller in the simulation. This can be used to allow
 * CTRE/REV sim to work with the sim library.
 */
public interface SimMotorController {
  public sealed interface ControllerOutput {
    public enum ControllerOutputTypes {
      VOLTAGE,
      CURRENT;

      public static final Struct<ControllerOutputTypes> struct =
          ProceduralStructGenerator.genEnum(ControllerOutputTypes.class);
    }

    public ControllerOutputTypes type();

    public record VoltageOutput(Voltage voltage) implements ControllerOutput {
      @Override
      public ControllerOutputTypes type() {
        return ControllerOutputTypes.VOLTAGE;
      }

      public static VoltageOutput of(double voltage) {
        return new VoltageOutput(Volts.of(voltage));
      }
    }

    public record CurrentOutput(Current current) implements ControllerOutput {
      @Override
      public ControllerOutputTypes type() {
        return ControllerOutputTypes.CURRENT;
      }

      public static CurrentOutput of(double current) {
        return new CurrentOutput(Amps.of(current));
      }
    }

    public static VoltageOutput of(Voltage voltage) {
      return new VoltageOutput(voltage);
    }

    public static CurrentOutput of(Current current) {
      return new CurrentOutput(current);
    }

    public static ControllerOutput zero() {
      return of(Amps.of(0));
    }

    public default double signumMagnitude() {
      if (this instanceof VoltageOutput) {
        return Math.signum(((VoltageOutput) this).voltage().baseUnitMagnitude());
      } else {
        return Math.signum(((CurrentOutput) this).current().baseUnitMagnitude());
      }
    }
  }

  /**
   * Runs the simulation for the motor.
   *
   * @param dt the time step, this will not be different between invocations of this method on the
   *     same instance
   * @param supply the supply voltage
   * @param state the current state of the mechanism, this is the state <b>at the rotor</b>. If you
   *     wish to scale these states based on the gearbox ratio, you can do
   *     `state.times(gearboxRatio)`.
   * @return the output voltage
   */
  ControllerOutput run(Time dt, Voltage supply, MechanismState state);

  /**
   * Returns whether the brake is enabled.
   *
   * @return whether the brake is enabled
   */
  boolean brakeEnabled();

  default void configureMotorModel(DCMotorExt motor) {}

  /**
   * Returns a motor controller that does nothing.
   *
   * @return a motor controller that does nothing
   */
  public static SimMotorController none() {
    return new SimMotorController() {
      @Override
      public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
        return new ControllerOutput.VoltageOutput(Volts.of(0));
      }

      @Override
      public boolean brakeEnabled() {
        return false;
      }

      @Override
      public void configureMotorModel(DCMotorExt motor) {}
    };
  }
}
