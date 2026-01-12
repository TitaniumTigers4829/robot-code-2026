package frc.robot.sim.simController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.math.mathutils.MeasureMath;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.extras.util.ProceduralStructGenerator;
import frc.robot.sim.simController.SimMotorController.ControllerOutput.CurrentOutput;
import frc.robot.sim.simController.SimMotorController.ControllerOutput.VoltageOutput;
import frc.robot.sim.simController.UnitSafeControl.TrapezoidProfile.State;
import frc.robot.sim.simMechanism.SimMechanism.MechanismState;
import java.util.Optional;

/**
 * A unit-safe motor controller for simulation that supports both closed-loop and open-loop control.
 * This controller applies soft limits and current limiting based on a provided motor model and
 * configuration parameters.
 */
public class UnitSafeMotorController implements SimMotorController {
  private static final VelocityUnit<AngleUnit> VU = VelocityUnit.combine(Radians, Seconds);
  private static final VelocityUnit<AngularVelocityUnit> AU =
      VelocityUnit.combine(RadiansPerSecond, Seconds);

  /** Represents an output command from the motor controller. */
  private sealed interface Output {
    /**
     * Computes the controller output for the given time step, supply voltage, and mechanism state.
     *
     * @param dt the time step for this update.
     * @param supply the available supply voltage.
     * @param state the current state of the mechanism.
     * @return the controller output as a {@link ControllerOutput}.
     */
    public ControllerOutput run(Time dt, Voltage supply, MechanismState state);

    /**
     * A closed-loop output that uses a provided closed-loop controller to compute a control value.
     *
     * @param <U> the unit type for the output value.
     */
    public record ClosedLoopOutput<U extends Unit>(
        ClosedLoop<?, U, AngleUnit> controller, Measure<U> value, Velocity<U> secondOrderValue)
        implements Output {

      /**
       * Runs the closed-loop controller by calculating the control output and error based on the
       * current mechanism state and desired setpoint.
       *
       * <p>If the controller is in velocity mode, it uses the velocity and acceleration values.
       * Otherwise, it uses the position and velocity values.
       *
       * @param dt the elapsed time step.
       * @param supply the available supply voltage.
       * @param state the current mechanism state.
       * @return the computed controller output as a {@link ControllerOutput} (voltage or current).
       */
      @Override
      @SuppressWarnings("unchecked")
      public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
        Measure<?> output;
        if (controller().isVelocity()) {
          output =
              controller()
                  .runVelocity(
                      state.position(),
                      State.of(state.velocity(), state.acceleration()),
                      new State<>(
                          (Measure<AngularVelocityUnit>) value,
                          (Velocity<AngularVelocityUnit>) secondOrderValue),
                      dt);
        } else {
          output =
              controller()
                  .runPosition(
                      state.position(),
                      State.of(state.position(), state.velocity()),
                      new State<>(
                          (Measure<AngleUnit>) value, (Velocity<AngleUnit>) secondOrderValue),
                      dt);
        }
        if (output.unit() instanceof VoltageUnit) {
          return ControllerOutput.of((Voltage) output);
        } else {
          return ControllerOutput.of((Current) output);
        }
      }
    }

    /** Represents an open-loop voltage command. */
    public record OpenLoopVoltageOutput(Voltage volts) implements Output {
      /**
       * Runs the open-loop voltage command by clamping the voltage to the available supply.
       *
       * @param dt the elapsed time step (unused in open-loop voltage).
       * @param supply the available supply voltage.
       * @param state the current mechanism state (unused).
       * @return the clamped voltage as a {@link ControllerOutput}.
       */
      @Override
      public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
        return ControllerOutput.of(MeasureMath.clamp(volts, supply));
      }
    }

    /** Represents an open-loop current command. */
    public record OpenLoopCurrentOutput(Current amps) implements Output {
      /**
       * Runs the open-loop current command.
       *
       * @param dt the elapsed time step (unused in open-loop current).
       * @param supply the available supply voltage (unused).
       * @param state the current mechanism state (unused).
       * @return the specified current as a {@link ControllerOutput}.
       */
      @Override
      public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
        return ControllerOutput.of(amps);
      }
    }

    /**
     * Creates an open-loop voltage output.
     *
     * @param volts the voltage command.
     * @return a new {@link OpenLoopVoltageOutput} with the given voltage.
     */
    public static OpenLoopVoltageOutput of(Voltage volts) {
      return new OpenLoopVoltageOutput(volts);
    }

    /**
     * Creates an open-loop current output.
     *
     * @param amps the current command.
     * @return a new {@link OpenLoopCurrentOutput} with the given current.
     */
    public static OpenLoopCurrentOutput of(Current amps) {
      return new OpenLoopCurrentOutput(amps);
    }
  }

  /**
   * Represents current limit parameters for the motor controller.
   *
   * <p>This record holds limits for the motor’s stator current, supply current, a lower supply
   * current limit, and a trigger time for the lower supply limit.
   *
   * @param statorCurrentLimit the maximum allowable stator current.
   * @param supplyCurrentLimit the maximum allowable supply current.
   * @param supplyCurrentLowerLimit the reduced supply current limit triggered after a set time.
   * @param lowerLimitTriggerTime the time after which the lower supply current limit is applied.
   */
  public record CurrentLimits(
      Current statorCurrentLimit,
      Current supplyCurrentLimit,
      Current supplyCurrentLowerLimit,
      Time lowerLimitTriggerTime)
      implements StructSerializable {

    /**
     * Returns the default base current limits.
     *
     * @return a {@link CurrentLimits} instance with default values.
     */
    public static CurrentLimits base() {
      return new CurrentLimits(Amps.of(120.0), Amps.of(70.0), Amps.of(40.0), Seconds.of(1.0));
    }

    /**
     * Scales the current limits by the given factor.
     *
     * @param factor the scaling factor.
     * @return a new {@link CurrentLimits} instance with scaled current limits.
     */
    public CurrentLimits times(double factor) {
      return new CurrentLimits(
          statorCurrentLimit.times(factor),
          supplyCurrentLimit.times(factor),
          supplyCurrentLowerLimit.times(factor),
          lowerLimitTriggerTime);
    }

    /** Structure definition for serialization. */
    public static final Struct<CurrentLimits> struct =
        ProceduralStructGenerator.genRecord(CurrentLimits.class);
  }

  /** Enumerates the possible soft limit trigger states. */
  public enum SoftLimitTrigger {
    NONE,
    REVERSE,
    FORWARD;

    /** Structure definition for serialization. */
    public static final Struct<SoftLimitTrigger> struct =
        ProceduralStructGenerator.genEnum(SoftLimitTrigger.class);
  }

  // Private instance fields

  /** The motor model used for simulation. */
  private DCMotorExt motor = null;

  /** The number of motors in the simulation (used for scaling current limits). */
  private int numMotors = 0;

  /** The current limits for the motor controller. */
  private CurrentLimits currentLimits = CurrentLimits.base();

  /** Accumulated time over the supply current limit. */
  private Time timeOverSupplyLimit = Seconds.of(0.0);

  /** The ratio between sensor measurements and the actual mechanism. */
  private double sensorToMechanismRatio = 1.0;

  /** The forward soft limit for mechanism position. */
  private Angle forwardSoftLimit = Radians.of(Double.POSITIVE_INFINITY);

  /** The reverse soft limit for mechanism position. */
  private Angle reverseSoftLimit = Radians.of(Double.NEGATIVE_INFINITY);

  /** The current output command, if any. */
  private Optional<Output> output = Optional.empty();

  /** Whether brake mode is enabled. */
  private boolean brakeMode = false;

  /** The last measured motor current. */
  private Current lastCurrent = Amps.of(0.0);

  /** The last calculated motor voltage. */
  private Voltage lastVoltage = Volts.of(0.0);

  /** The last mechanism state. */
  private MechanismState lastState = MechanismState.zero();

  /** Constructs a new UnitSafeMotorController. */
  public UnitSafeMotorController() {}

  /**
   * Configures the current limits for the motor controller.
   *
   * @param currentLimit the current limit configuration.
   * @return this UnitSafeMotorController instance.
   */
  public UnitSafeMotorController configureCurrentLimit(CurrentLimits currentLimit) {
    this.currentLimits = currentLimit;
    return this;
  }

  /**
   * Configures the soft limits for mechanism position.
   *
   * @param forwardLimit the forward position limit.
   * @param reverseLimit the reverse position limit.
   * @return this UnitSafeMotorController instance.
   */
  public UnitSafeMotorController configureSoftLimits(Angle forwardLimit, Angle reverseLimit) {
    this.forwardSoftLimit = forwardLimit;
    this.reverseSoftLimit = reverseLimit;
    return this;
  }

  /**
   * Configures the sensor-to-mechanism ratio.
   *
   * @param ratio the ratio from sensor measurement to mechanism units.
   * @return this UnitSafeMotorController instance.
   */
  public UnitSafeMotorController configSensorToMechanismRatio(double ratio) {
    this.sensorToMechanismRatio = ratio;
    return this;
  }

  /**
   * Configures the motor model used for simulation.
   *
   * @param motor the motor model.
   */
  @Override
  public void configureMotorModel(DCMotorExt motor) {
    this.motor = motor;
    this.numMotors = motor.numMotors;
  }

  /**
   * Sets the brake mode state.
   *
   * @param brakeMode {@code true} to enable brake mode; {@code false} otherwise.
   */
  public void setBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }

  /**
   * Sets closed-loop current control using position and velocity feedback.
   *
   * @param controller the closed-loop controller for current.
   * @param position the desired mechanism position.
   * @param velocity the measured mechanism velocity.
   */
  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> controller,
      Angle position,
      AngularVelocity velocity) {
    if (controller == null || position == null) {
      output = Optional.empty();
      return;
    }
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, position, VU.of(velocity.baseUnitMagnitude())));
  }

  /**
   * Sets closed-loop current control using position feedback only.
   *
   * @param controller the closed-loop controller for current.
   * @param position the desired mechanism position.
   */
  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> controller, Angle position) {
    controlCurrent(controller, position, RadiansPerSecond.zero());
  }

  /**
   * Sets closed-loop current control using velocity and acceleration feedback.
   *
   * @param controller the closed-loop controller for current.
   * @param velocity the desired mechanism velocity.
   * @param acceleration the desired mechanism acceleration.
   */
  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {
    if (controller == null || velocity == null) {
      output = Optional.empty();
      return;
    }
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, velocity, AU.of(acceleration.baseUnitMagnitude())));
  }

  /**
   * Sets closed-loop current control using velocity feedback only.
   *
   * @param controller the closed-loop controller for current.
   * @param velocity the desired mechanism velocity.
   */
  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity) {
    controlCurrent(controller, velocity, RadiansPerSecondPerSecond.zero());
  }

  /**
   * Sets open-loop current control.
   *
   * @param amps the current command.
   */
  public void controlCurrent(Current amps) {
    output = Optional.of(Output.of(amps));
  }

  /**
   * Sets closed-loop voltage control using position and velocity feedback.
   *
   * @param controller the closed-loop controller for voltage.
   * @param position the desired mechanism position.
   * @param velocity the measured mechanism velocity.
   */
  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> controller,
      Angle position,
      AngularVelocity velocity) {
    if (controller == null || position == null) {
      output = Optional.empty();
      return;
    }
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, position, VU.of(velocity.baseUnitMagnitude())));
  }

  /**
   * Sets closed-loop voltage control using position feedback only.
   *
   * @param controller the closed-loop controller for voltage.
   * @param position the desired mechanism position.
   */
  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> controller, Angle position) {
    controlVoltage(controller, position, RadiansPerSecond.zero());
  }

  /**
   * Sets closed-loop voltage control using velocity and acceleration feedback.
   *
   * @param controller the closed-loop controller for voltage.
   * @param velocity the desired mechanism velocity.
   * @param acceleration the desired mechanism acceleration.
   */
  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {
    if (controller == null || velocity == null) {
      output = Optional.empty();
      return;
    }
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, velocity, AU.of(acceleration.baseUnitMagnitude())));
  }

  /**
   * Sets closed-loop voltage control using velocity feedback only.
   *
   * @param controller the closed-loop controller for voltage.
   * @param velocity the desired mechanism velocity.
   */
  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity) {
    controlVoltage(controller, velocity, RadiansPerSecondPerSecond.zero());
  }

  /**
   * Sets open-loop voltage control.
   *
   * @param volts the voltage command.
   */
  public void controlVoltage(Voltage volts) {
    output = Optional.of(Output.of(volts));
  }

  /**
   * Returns the last computed mechanism position.
   *
   * @return the current position.
   */
  public Angle position() {
    return lastState.position();
  }

  /**
   * Returns the last computed mechanism velocity.
   *
   * @return the current velocity.
   */
  public AngularVelocity velocity() {
    return lastState.velocity();
  }

  /**
   * Returns the last computed mechanism acceleration.
   *
   * @return the current acceleration.
   */
  public AngularAcceleration acceleration() {
    return lastState.acceleration();
  }

  /**
   * Returns the last measured stator current.
   *
   * @return the current.
   */
  public Current current() {
    return lastCurrent;
  }

  /**
   * Returns the last computed motor voltage.
   *
   * @return the voltage.
   */
  public Voltage voltage() {
    return lastVoltage;
  }

  /**
   * Indicates whether brake mode is enabled.
   *
   * @return {@code true} if brake mode is enabled; {@code false} otherwise.
   */
  @Override
  public boolean brakeEnabled() {
    return brakeMode;
  }

  /**
   * Executes one simulation step by calculating the desired output based on the current control
   * mode and then applying soft limits and current limits.
   *
   * @param dt the elapsed time step.
   * @param supply the available supply voltage.
   * @param rawState the raw mechanism state.
   * @return the final {@link ControllerOutput} after applying all limits.
   */
  @Override
  public ControllerOutput run(Time dt, Voltage supply, MechanismState rawState) {
    // Adjust the raw mechanism state based on the sensor-to-mechanism ratio.
    MechanismState state = rawState.div(sensorToMechanismRatio);
    lastState = state;
    return output
        .map(o -> o.run(dt, supply, state))
        .map(o -> softLimit(o, state))
        .map(o -> currentLimit(dt, o, state, supply))
        .orElseGet(() -> ControllerOutput.zero());
  }

  /**
   * Applies soft limit constraints to the requested output based on the mechanism's position.
   *
   * @param requestedOutput the controller output requested.
   * @param state the current mechanism state.
   * @return a zero output if the soft limits are exceeded; otherwise, the original requested
   *     output.
   */
  private ControllerOutput softLimit(ControllerOutput requestedOutput, MechanismState state) {
    double direction = requestedOutput.signumMagnitude();
    Angle position = state.position();
    if (direction > 0 && position.in(Radians) > forwardSoftLimit.in(Radians)) {
      return ControllerOutput.zero();
    } else if (direction < 0 && position.in(Radians) < reverseSoftLimit.in(Radians)) {
      return ControllerOutput.zero();
    }
    return requestedOutput;
  }

  /**
   * Applies current limiting logic based on motor characteristics and supply limits. This method
   * adjusts the voltage or current output if any of the current limits are exceeded.
   *
   * @param dt the elapsed time step.
   * @param requestedOutput the initial controller output.
   * @param state the current mechanism state.
   * @param supplyVoltage the available supply voltage.
   * @return the adjusted {@link ControllerOutput} after current limiting.
   */
  private ControllerOutput currentLimit(
      Time dt, ControllerOutput requestedOutput, MechanismState state, Voltage supplyVoltage) {
    // See https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3) for
    // reference.
    final CurrentLimits limits = currentLimits.times(numMotors);
    final AngularVelocity velocity = state.velocity();
    Voltage voltageInput;
    Current statorCurrent;
    Current supplyCurrent;

    if (requestedOutput instanceof VoltageOutput vo) {
      voltageInput = vo.voltage();
      statorCurrent = motor.getCurrent(velocity, voltageInput);
    } else {
      CurrentOutput co = (CurrentOutput) requestedOutput;
      statorCurrent = co.current();
      voltageInput = motor.getVoltage(statorCurrent, velocity);
    }

    voltageInput = MeasureMath.clamp(voltageInput, supplyVoltage);
    supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);

    boolean overStatorLimit = statorCurrent.gt(limits.statorCurrentLimit);
    boolean overSupplyLimit = supplyCurrent.gt(limits.supplyCurrentLimit);
    boolean overLowerSupplyLimit =
        supplyCurrent.gt(limits.supplyCurrentLowerLimit)
            && timeOverSupplyLimit.gt(limits.lowerLimitTriggerTime);

    if (overStatorLimit) {
      statorCurrent = limits.statorCurrentLimit;
      voltageInput = motor.getVoltage(statorCurrent, velocity);
      supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);
    }

    if (overSupplyLimit) {
      timeOverSupplyLimit = timeOverSupplyLimit.plus(dt);
      Current supplyLimit = limits.supplyCurrentLimit;
      if (timeOverSupplyLimit.gt(limits.lowerLimitTriggerTime)) {
        supplyLimit = limits.supplyCurrentLowerLimit;
      }
      statorCurrent =
          motor.getCurrent(velocity, voltageInput, supplyLimit, limits.statorCurrentLimit);
      voltageInput = motor.getVoltage(statorCurrent, velocity);
    } else {
      timeOverSupplyLimit = Seconds.of(0.0);
    }

    lastVoltage = voltageInput;
    lastCurrent = statorCurrent;

    if (requestedOutput instanceof VoltageOutput) {
      return ControllerOutput.of(voltageInput);
    } else {
      return ControllerOutput.of(statorCurrent);
    }
  }
}
