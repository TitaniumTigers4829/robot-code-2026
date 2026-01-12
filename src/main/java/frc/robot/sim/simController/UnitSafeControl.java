package frc.robot.sim.simController;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.math.mathutils.MeasureMath;
import frc.robot.extras.util.ProceduralStructGenerator;
import org.littletonrobotics.junction.Logger;

/**
 * Provides unit-safe control implementations including PID feedback controllers, feedforward
 * controllers, and trapezoidal motion profiles.
 */
public class UnitSafeControl {

  /**
   * A PID feedback controller that leverages units to ensure the controller is used correctly.
   *
   * @param <O> The unit type for the controller output.
   * @param <Q> The unit type for the controller input (measurement and setpoint).
   */
  public static class PIDFeedback<O extends Unit, Q extends Unit> {
    private final PIDController internalController;
    private final O outputUnit;
    private final Q inputUnit;

    /**
     * Constructs a PIDFeedback controller with proportional, integral, and derivative gains.
     *
     * @param kP the proportional gain as a ratio of output to input.
     * @param kI the integral gain as a ratio of output to input.
     * @param kD the derivative gain as a ratio of output to the velocity of input.
     */
    public PIDFeedback(Per<O, Q> kP, Per<O, Q> kI, Per<O, VelocityUnit<Q>> kD) {
      outputUnit = kP.unit().numerator();
      inputUnit = kP.unit().denominator();
      internalController =
          new PIDController(kP.baseUnitMagnitude(), kI.baseUnitMagnitude(), kD.baseUnitMagnitude());
    }

    /**
     * Constructs a PIDFeedback controller with proportional and derivative gains. The integral gain
     * is set to zero.
     *
     * @param kP the proportional gain as a ratio of output to input.
     * @param kD the derivative gain as a ratio of output to the velocity of input.
     */
    public PIDFeedback(Per<O, Q> kP, Per<O, VelocityUnit<Q>> kD) {
      outputUnit = kP.unit().numerator();
      inputUnit = kP.unit().denominator();
      internalController = new PIDController(kP.baseUnitMagnitude(), 0.0, kD.baseUnitMagnitude());
    }

    /**
     * Constructs a PIDFeedback controller with only a proportional gain. The integral and
     * derivative gains are set to zero.
     *
     * @param kP the proportional gain as a ratio of output to input.
     */
    public PIDFeedback(Per<O, Q> kP) {
      outputUnit = kP.unit().numerator();
      inputUnit = kP.unit().denominator();
      internalController = new PIDController(kP.baseUnitMagnitude(), 0.0, 0.0);
    }

    /** Logs the current PID controller gains and unit configuration. */
    public void logGains() {
      Logger.recordOutput("Sim/kP", internalController.getP());
      Logger.recordOutput("Sim/kI", internalController.getI());
      Logger.recordOutput("Sim/kD", internalController.getD());
      Logger.recordOutput("Sim/outputUnit", outputUnit.name());
      Logger.recordOutput("Sim/inputUnit", inputUnit.name());
    }

    /**
     * Calculates the PID control output and error based on the provided measurement and setpoint.
     *
     * @param measurement the current measurement.
     * @param setpoint the desired setpoint.
     * @return a Pair where the first element is the control output and the second element is the
     *     error.
     */
    @SuppressWarnings("unchecked")
    public Pair<Measure<O>, Measure<Q>> calculate(Measure<Q> measurement, Measure<Q> setpoint) {
      var o =
          (Measure<O>)
              outputUnit.of(
                  internalController.calculate(
                      measurement.baseUnitMagnitude(), setpoint.baseUnitMagnitude()));
      var error = (Measure<Q>) inputUnit.of(internalController.getError());
      return new Pair<>(o, error);
    }

    /**
     * Sets the acceptable error tolerance for the controller.
     *
     * @param tolerance the tolerance for the measurement error.
     * @return this PIDFeedback instance.
     */
    public PIDFeedback<O, Q> withTolerance(Measure<Q> tolerance) {
      internalController.setTolerance(tolerance.baseUnitMagnitude());
      return this;
    }

    /**
     * Sets the acceptable position and velocity error tolerances for the controller.
     *
     * @param positionTolerance the tolerance for the position error.
     * @param velocityTolerance the tolerance for the velocity error.
     * @return this PIDFeedback instance.
     */
    public PIDFeedback<O, Q> withTolerance(
        Measure<Q> positionTolerance, Measure<Q> velocityTolerance) {
      internalController.setTolerance(
          positionTolerance.baseUnitMagnitude(), velocityTolerance.baseUnitMagnitude());
      return this;
    }

    /**
     * Enables continuous input mode, treating the input range as circular.
     *
     * @param minimumInput the minimum input value.
     * @param maximumInput the maximum input value.
     * @return this PIDFeedback instance.
     */
    public PIDFeedback<O, Q> withContinuousInput(Measure<Q> minimumInput, Measure<Q> maximumInput) {
      internalController.enableContinuousInput(
          minimumInput.baseUnitMagnitude(), maximumInput.baseUnitMagnitude());
      return this;
    }

    /**
     * Gets the output unit used by this controller.
     *
     * @return the output unit.
     */
    O getOutputUnit() {
      return outputUnit;
    }

    /**
     * Gets the input unit used by this controller.
     *
     * @return the input unit.
     */
    Q getInputUnit() {
      return inputUnit;
    }
  }

  /**
   * A PID feedback controller specialized for linear (distance) control.
   *
   * @param <O> The unit type for the controller output.
   */
  public static class LinearPIDFeedback<O extends Unit> extends PIDFeedback<O, DistanceUnit> {

    /**
     * Constructs a LinearPIDFeedback controller with proportional, integral, and derivative gains.
     *
     * @param kP the proportional gain as a ratio of output to distance.
     * @param kI the integral gain as a ratio of output to distance.
     * @param kD the derivative gain as a ratio of output to velocity (per unit time) of distance.
     */
    public LinearPIDFeedback(
        Per<O, DistanceUnit> kP,
        Per<O, DistanceUnit> kI,
        Per<O, PerUnit<DistanceUnit, TimeUnit>> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Meters, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs a LinearPIDFeedback controller with proportional and derivative gains.
     *
     * @param kP the proportional gain as a ratio of output to distance.
     * @param kD the derivative gain as a ratio of output to velocity (per unit time) of distance.
     */
    public LinearPIDFeedback(Per<O, DistanceUnit> kP, Per<O, PerUnit<DistanceUnit, TimeUnit>> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Meters, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs a LinearPIDFeedback controller with only a proportional gain.
     *
     * @param kP the proportional gain as a ratio of output to distance.
     */
    public LinearPIDFeedback(Per<O, DistanceUnit> kP) {
      super(kP);
    }

    /**
     * Calculates the control output and error for a given linear measurement and setpoint.
     *
     * @param measurement the current distance measurement.
     * @param setpoint the desired distance setpoint.
     * @return a Pair where the first element is the control output and the second element is the
     *     error.
     */
    public Pair<Measure<O>, Measure<DistanceUnit>> calculate(
        Distance measurement, Distance setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  /**
   * A PID feedback controller specialized for linear velocity control.
   *
   * @param <O> The unit type for the controller output.
   */
  public static class LinearVelocityPIDFeedback<O extends Unit>
      extends PIDFeedback<O, LinearVelocityUnit> {

    /**
     * Constructs a LinearVelocityPIDFeedback controller with proportional, integral, and derivative
     * gains.
     *
     * @param kP the proportional gain as a ratio of output to linear velocity.
     * @param kI the integral gain as a ratio of output to linear velocity.
     * @param kD the derivative gain as a ratio of output to linear acceleration.
     */
    public LinearVelocityPIDFeedback(
        Per<O, LinearVelocityUnit> kP,
        Per<O, LinearVelocityUnit> kI,
        Per<O, LinearAccelerationUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(MetersPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs a LinearVelocityPIDFeedback controller with proportional and derivative gains.
     *
     * @param kP the proportional gain as a ratio of output to linear velocity.
     * @param kD the derivative gain as a ratio of output to linear acceleration.
     */
    public LinearVelocityPIDFeedback(
        Per<O, LinearVelocityUnit> kP, Per<O, PerUnit<LinearVelocityUnit, TimeUnit>> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(MetersPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs a LinearVelocityPIDFeedback controller with only a proportional gain.
     *
     * @param kP the proportional gain as a ratio of output to linear velocity.
     */
    public LinearVelocityPIDFeedback(Per<O, LinearVelocityUnit> kP) {
      super(kP);
    }

    /**
     * Calculates the control output and error for a given linear velocity measurement and setpoint.
     *
     * @param measurement the current linear velocity measurement.
     * @param setpoint the desired linear velocity setpoint.
     * @return a Pair where the first element is the control output and the second element is the
     *     error.
     */
    public Pair<Measure<O>, Measure<LinearVelocityUnit>> calculate(
        LinearVelocity measurement, LinearVelocity setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  /**
   * A PID feedback controller specialized for angular control.
   *
   * @param <O> The unit type for the controller output.
   */
  public static class AngularPIDFeedback<O extends Unit> extends PIDFeedback<O, AngleUnit> {

    /**
     * Constructs an AngularPIDFeedback controller with proportional, integral, and derivative
     * gains.
     *
     * @param kP the proportional gain as a ratio of output to angle.
     * @param kI the integral gain as a ratio of output to angle.
     * @param kD the derivative gain as a ratio of output to angular velocity.
     */
    public AngularPIDFeedback(
        Per<O, AngleUnit> kP, Per<O, AngleUnit> kI, Per<O, AngularVelocityUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Radian, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs an AngularPIDFeedback controller with proportional and derivative gains.
     *
     * @param kP the proportional gain as a ratio of output to angle.
     * @param kD the derivative gain as a ratio of output to angular velocity.
     */
    public AngularPIDFeedback(Per<O, AngleUnit> kP, Per<O, AngularVelocityUnit> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(Radian, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs an AngularPIDFeedback controller with only a proportional gain.
     *
     * @param kP the proportional gain as a ratio of output to angle.
     */
    public AngularPIDFeedback(Per<O, AngleUnit> kP) {
      super(kP);
    }

    /**
     * Calculates the control output and error for a given angular measurement and setpoint.
     *
     * @param measurement the current angular measurement.
     * @param setpoint the desired angular setpoint.
     * @return a Pair where the first element is the control output and the second element is the
     *     error.
     */
    public Pair<Measure<O>, Measure<AngleUnit>> calculate(Angle measurement, Angle setpoint) {
      return super.calculate(measurement, setpoint);
    }

    /**
     * Configures the controller to use continuous angular input over the range [-pi, pi].
     *
     * @return this AngularPIDFeedback instance.
     */
    public AngularPIDFeedback<O> withContinuousAngularInput() {
      super.withContinuousInput(Radian.of(-Math.PI), Radian.of(Math.PI));
      return this;
    }
  }

  /**
   * A PID feedback controller specialized for angular velocity control.
   *
   * @param <O> The unit type for the controller output.
   */
  public static class AngularVelocityPIDFeedback<O extends Unit>
      extends PIDFeedback<O, AngularVelocityUnit> {

    /**
     * Constructs an AngularVelocityPIDFeedback controller with proportional, integral, and
     * derivative gains.
     *
     * @param kP the proportional gain as a ratio of output to angular velocity.
     * @param kI the integral gain as a ratio of output to angular velocity.
     * @param kD the derivative gain as a ratio of output to angular acceleration.
     */
    public AngularVelocityPIDFeedback(
        Per<O, AngularVelocityUnit> kP,
        Per<O, AngularVelocityUnit> kI,
        Per<O, AngularAccelerationUnit> kD) {
      super(
          kP,
          kI,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(RadiansPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs an AngularVelocityPIDFeedback controller with proportional and derivative gains.
     *
     * @param kP the proportional gain as a ratio of output to angular velocity.
     * @param kD the derivative gain as a ratio of output to angular acceleration.
     */
    public AngularVelocityPIDFeedback(
        Per<O, AngularVelocityUnit> kP, Per<O, AngularAccelerationUnit> kD) {
      super(
          kP,
          PerUnit.combine(kD.unit().numerator(), VelocityUnit.combine(RadiansPerSecond, Second))
              .ofNative(kD.baseUnitMagnitude()));
    }

    /**
     * Constructs an AngularVelocityPIDFeedback controller with only a proportional gain.
     *
     * @param kP the proportional gain as a ratio of output to angular velocity.
     */
    public AngularVelocityPIDFeedback(Per<O, AngularVelocityUnit> kP) {
      super(kP);
    }

    /**
     * Calculates the control output and error for a given angular velocity measurement and
     * setpoint.
     *
     * @param measurement the current angular velocity measurement.
     * @param setpoint the desired angular velocity setpoint.
     * @return a Pair where the first element is the control output and the second element is the
     *     error.
     */
    public Pair<Measure<O>, Measure<AngularVelocityUnit>> calculate(
        AngularVelocity measurement, AngularVelocity setpoint) {
      return super.calculate(measurement, setpoint);
    }
  }

  /**
   * Interface representing a feedforward controller.
   *
   * @param <O> The unit type for the controller output.
   * @param <Q> The unit type for the controller input.
   */
  public sealed interface Feedforward<O extends Unit, Q extends Unit>
      permits FlywheelFeedforward, ElevatorFeedforward, ArmFeedforward {
    /** Logs the feedforward gains. */
    public void logGains();
  }

  /**
   * Feedforward controller for flywheel mechanisms.
   *
   * @param <O> The unit type for the controller output.
   */
  public static final class FlywheelFeedforward<O extends Unit>
      implements Feedforward<O, AngleUnit> {
    private final SimpleMotorFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS;
    final Per<O, AngularVelocityUnit> kV;
    final Per<O, AngularAccelerationUnit> kA;

    /**
     * Constructs a FlywheelFeedforward controller with static, velocity, and acceleration gains.
     *
     * @param kS the static gain.
     * @param kV the velocity gain as a ratio of output to angular velocity.
     * @param kA the acceleration gain as a ratio of output to angular acceleration.
     */
    public FlywheelFeedforward(
        Measure<O> kS, Per<O, AngularVelocityUnit> kV, Per<O, AngularAccelerationUnit> kA) {
      outputUnit = kS.unit();
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      internalFeedforward =
          new SimpleMotorFeedforward(
              kS.baseUnitMagnitude(), kV.baseUnitMagnitude(), kA.baseUnitMagnitude());
    }

    /**
     * Constructs a FlywheelFeedforward controller with only a static gain.
     *
     * @param kS the static gain.
     */
    public FlywheelFeedforward(Measure<O> kS) {
      outputUnit = kS.unit();
      this.kS = kS;
      this.kV = null;
      this.kA = null;
      internalFeedforward = new SimpleMotorFeedforward(kS.baseUnitMagnitude(), 0.0, 0.0);
    }

    @Override
    public void logGains() {
      Logger.recordOutput("Sim/kS", kS.baseUnitMagnitude());
      if (kV != null) {
        Logger.recordOutput("Sim/kV", kV.baseUnitMagnitude());
      }
      if (kA != null) {
        Logger.recordOutput("Sim/kA", kA.baseUnitMagnitude());
      }
      Logger.recordOutput("Sim/outputUnit", outputUnit.name());
      Logger.recordOutput("Sim/inputUnit", Radian.name());
      Logger.recordOutput("Sim/feedforwardType", "Flywheel");
    }

    /**
     * Calculates the feedforward output based on the desired angular velocity and angular
     * acceleration.
     *
     * @param goalRate the desired angular velocity.
     * @param goalRateRate the desired angular acceleration.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings({"unchecked", "removal"})
    public Measure<O> calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
    }

    /**
     * Calculates the feedforward output using current and next angular velocity values.
     *
     * @param goalRate the current desired angular velocity.
     * @param nextGoalRate the next desired angular velocity.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings("unchecked")
    public Measure<O> calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  goalRate.baseUnitMagnitude(), nextGoalRate.baseUnitMagnitude()));
    }

    /**
     * Calculates the feedforward output based on the desired angular velocity.
     *
     * @param goalRate the desired angular velocity.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(AngularVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(internalFeedforward.calculate(goalRate.baseUnitMagnitude()));
    }
  }

  /**
   * Feedforward controller for elevator mechanisms.
   *
   * @param <O> The unit type for the controller output.
   */
  public static final class ElevatorFeedforward<O extends Unit>
      implements Feedforward<O, DistanceUnit> {
    private final edu.wpi.first.math.controller.ElevatorFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS;
    final Per<O, VelocityUnit<DistanceUnit>> kV;
    final Per<O, AccelerationUnit<DistanceUnit>> kA;

    /**
     * Constructs an ElevatorFeedforward controller with static, velocity, and acceleration gains.
     *
     * @param kS the static gain.
     * @param kV the velocity gain as a ratio of output to linear velocity.
     * @param kA the acceleration gain as a ratio of output to linear acceleration.
     */
    public ElevatorFeedforward(
        Measure<O> kS,
        Per<O, VelocityUnit<DistanceUnit>> kV,
        Per<O, AccelerationUnit<DistanceUnit>> kA) {
      outputUnit = kS.unit();
      internalFeedforward =
          new edu.wpi.first.math.controller.ElevatorFeedforward(
              kS.baseUnitMagnitude(), kV.baseUnitMagnitude(), kA.baseUnitMagnitude());
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }

    @Override
    public void logGains() {
      Logger.recordOutput("Sim/kS", kS.baseUnitMagnitude());
      Logger.recordOutput("Sim/kV", kV.baseUnitMagnitude());
      Logger.recordOutput("Sim/kA", kA.baseUnitMagnitude());
      Logger.recordOutput("Sim/outputUnit", outputUnit.name());
      Logger.recordOutput("Sim/inputUnit", Meters.name());
      Logger.recordOutput("Sim/feedforwardType", "Elevator");
    }

    /**
     * Calculates the feedforward output based on the desired linear velocity and linear
     * acceleration.
     *
     * @param goalRate the desired linear velocity.
     * @param goalRateRate the desired linear acceleration.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings({"unchecked", "removal"})
    public Measure<O> calculate(LinearVelocity goalRate, LinearAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
    }

    /**
     * Calculates the feedforward output using current and next linear velocity values.
     *
     * @param goalRate the current desired linear velocity.
     * @param nextGoalRate the next desired linear velocity.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings("unchecked")
    public Measure<O> calculate(LinearVelocity goalRate, LinearVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  goalRate.baseUnitMagnitude(), nextGoalRate.baseUnitMagnitude()));
    }

    /**
     * Calculates the feedforward output based on the desired linear velocity.
     *
     * @param goalRate the desired linear velocity.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(LinearVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(internalFeedforward.calculate(goalRate.baseUnitMagnitude()));
    }
  }

  /**
   * Feedforward controller for arm mechanisms.
   *
   * @param <O> The unit type for the controller output.
   */
  public static final class ArmFeedforward<O extends Unit> implements Feedforward<O, AngleUnit> {
    private final edu.wpi.first.math.controller.ArmFeedforward internalFeedforward;
    private final O outputUnit;
    final Measure<O> kS;
    final Per<O, VelocityUnit<AngleUnit>> kV;
    final Per<O, AccelerationUnit<AngleUnit>> kA;

    /**
     * Constructs an ArmFeedforward controller with static, velocity, and acceleration gains.
     *
     * @param kS the static gain.
     * @param kV the velocity gain as a ratio of output to angular velocity.
     * @param kA the acceleration gain as a ratio of output to angular acceleration.
     */
    public ArmFeedforward(
        Measure<O> kS, Per<O, VelocityUnit<AngleUnit>> kV, Per<O, AccelerationUnit<AngleUnit>> kA) {
      outputUnit = kS.unit();
      internalFeedforward =
          new edu.wpi.first.math.controller.ArmFeedforward(
              kS.baseUnitMagnitude(), kV.baseUnitMagnitude(), kA.baseUnitMagnitude());
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }

    /**
     * Calculates the feedforward output based on the current arm angle, desired angular velocity,
     * and angular acceleration.
     *
     * @param currentAngle the current arm angle.
     * @param goalRate the desired angular velocity.
     * @param goalRateRate the desired angular acceleration.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings("unchecked")
    public Measure<O> calculate(
        Angle currentAngle, AngularVelocity goalRate, AngularAcceleration goalRateRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  currentAngle.baseUnitMagnitude(),
                  goalRate.baseUnitMagnitude(),
                  goalRateRate.baseUnitMagnitude()));
    }

    /**
     * Calculates the feedforward output using current arm angle and two successive angular velocity
     * values.
     *
     * @param currentAngle the current arm angle.
     * @param goalRate the current desired angular velocity.
     * @param nextGoalRate the next desired angular velocity.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings("unchecked")
    public Measure<O> calculate(
        Angle currentAngle, AngularVelocity goalRate, AngularVelocity nextGoalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculateWithVelocities(
                  currentAngle.baseUnitMagnitude(),
                  goalRate.baseUnitMagnitude(),
                  nextGoalRate.baseUnitMagnitude()));
    }

    /**
     * Calculates the feedforward output based on the current arm angle and desired angular
     * velocity.
     *
     * @param currentAngle the current arm angle.
     * @param goalRate the desired angular velocity.
     * @return the calculated output as a measure in the output unit.
     */
    @SuppressWarnings({"unchecked"})
    public Measure<O> calculate(Angle currentAngle, AngularVelocity goalRate) {
      return (Measure<O>)
          outputUnit.of(
              internalFeedforward.calculate(
                  currentAngle.baseUnitMagnitude(), goalRate.baseUnitMagnitude()));
    }

    // TODO: Fill in.
    @Override
    public void logGains() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'logGains'");
    }
  }

  /**
   * Implements a trapezoidal motion profile for trajectory planning with unit safety.
   *
   * @param <DIM> The dimensional unit for the profile (e.g. distance, angle).
   */
  public static class TrapezoidProfile<DIM extends Unit> {
    private final edu.wpi.first.math.trajectory.TrapezoidProfile internalProfile;
    private final Measure<DIM> maxValue;
    private final Velocity<DIM> maxSlew;
    private final Acceleration<DIM> maxSlewSlew;

    /**
     * Represents a state within a trapezoidal motion profile, consisting of a value (position) and
     * a slew (velocity).
     *
     * @param <DIM> The dimensional unit for the state.
     * @param value the position value.
     * @param slew the velocity (slew) value.
     */
    public record State<DIM extends Unit>(Measure<DIM> value, Velocity<DIM> slew)
        implements StructSerializable {
      /**
       * Creates an angular state from the given position and velocity.
       *
       * @param position the angular position.
       * @param velocity the angular velocity.
       * @return a new angular State.
       */
      public static State<AngleUnit> of(Angle position, AngularVelocity velocity) {
        return new State<>(
            position, VelocityUnit.combine(Radian, Second).of(velocity.in(RadiansPerSecond)));
      }

      /**
       * Creates a distance state from the given position and velocity.
       *
       * @param position the distance position.
       * @param velocity the linear velocity.
       * @return a new distance State.
       */
      public static State<DistanceUnit> of(Distance position, LinearVelocity velocity) {
        return new State<>(
            position, VelocityUnit.combine(Meters, Second).of(velocity.in(MetersPerSecond)));
      }

      /**
       * Creates an angular velocity state from the given velocity and acceleration.
       *
       * @param velocity the angular velocity.
       * @param acceleration the angular acceleration.
       * @return a new angular velocity State.
       */
      public static State<AngularVelocityUnit> of(
          AngularVelocity velocity, AngularAcceleration acceleration) {
        return new State<>(
            velocity,
            VelocityUnit.combine(RadiansPerSecond, Second)
                .of(acceleration.in(RadiansPerSecondPerSecond)));
      }

      /**
       * Creates a linear velocity state from the given velocity and acceleration.
       *
       * @param velocity the linear velocity.
       * @param acceleration the linear acceleration.
       * @return a new linear velocity State.
       */
      public static State<LinearVelocityUnit> of(
          LinearVelocity velocity, LinearAcceleration acceleration) {
        return new State<>(
            velocity,
            VelocityUnit.combine(MetersPerSecond, Second)
                .of(acceleration.in(MetersPerSecondPerSecond)));
      }

      @SuppressWarnings("rawtypes")
      public static final Struct<State> struct = ProceduralStructGenerator.genRecord(State.class);
    }

    /**
     * Private constructor for the trapezoidal profile.
     *
     * @param maxValue the maximum allowable value.
     * @param maxSlew the maximum slew (velocity) value.
     * @param maxSlewSlew the maximum slew rate (acceleration) value.
     */
    private TrapezoidProfile(
        Measure<DIM> maxValue, Velocity<DIM> maxSlew, Acceleration<DIM> maxSlewSlew) {
      this.maxValue = maxValue;
      this.maxSlew = maxSlew;
      this.maxSlewSlew = maxSlewSlew;
      internalProfile =
          new edu.wpi.first.math.trajectory.TrapezoidProfile(
              new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                  maxSlew.baseUnitMagnitude(), maxSlewSlew.baseUnitMagnitude()));
    }

    /**
     * Creates a trapezoidal profile for angular motion.
     *
     * @param maxVelocity the maximum angular velocity.
     * @param maxAcceleration the maximum angular acceleration.
     * @return a TrapezoidProfile configured for angular motion.
     */
    public static TrapezoidProfile<AngleUnit> forAngle(
        AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
      final VelocityUnit<AngleUnit> vu = VelocityUnit.combine(Radian, Second);
      return new TrapezoidProfile<>(
          Radian.of(10000000000.0),
          vu.of(maxVelocity.in(RadiansPerSecond)),
          AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(RadiansPerSecondPerSecond)));
    }

    /**
     * Creates a trapezoidal profile for linear distance motion.
     *
     * @param maxVelocity the maximum linear velocity.
     * @param maxAcceleration the maximum linear acceleration.
     * @return a TrapezoidProfile configured for distance motion.
     */
    public static TrapezoidProfile<DistanceUnit> forDistance(
        LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
      final VelocityUnit<DistanceUnit> vu = VelocityUnit.combine(Meters, Second);
      return new TrapezoidProfile<>(
          Meters.of(10000000000.0),
          vu.of(maxVelocity.in(MetersPerSecond)),
          AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(MetersPerSecondPerSecond)));
    }

    /**
     * Creates a trapezoidal profile for angular velocity.
     *
     * @param maxVelocity the maximum angular velocity.
     * @param maxAcceleration the maximum angular acceleration.
     * @return a TrapezoidProfile configured for angular velocity.
     */
    public static TrapezoidProfile<AngularVelocityUnit> forAngularVelocity(
        AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
      final VelocityUnit<AngularVelocityUnit> vu = VelocityUnit.combine(RadiansPerSecond, Second);
      return new TrapezoidProfile<>(
          maxVelocity,
          vu.of(maxAcceleration.in(RadiansPerSecondPerSecond)),
          AccelerationUnit.combine(vu, Second).of(10000000.0));
    }

    /**
     * Creates a trapezoidal profile for linear velocity.
     *
     * @param maxVelocity the maximum linear velocity.
     * @param maxAcceleration the maximum linear acceleration.
     * @return a TrapezoidProfile configured for linear velocity.
     */
    public static TrapezoidProfile<LinearVelocityUnit> forLinearVelocity(
        LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
      final VelocityUnit<LinearVelocityUnit> vu = VelocityUnit.combine(MetersPerSecond, Second);
      return new TrapezoidProfile<>(
          maxVelocity,
          vu.of(maxAcceleration.in(MetersPerSecondPerSecond)),
          AccelerationUnit.combine(vu, Second).of(10000000.0));
    }

    /**
     * Logs the trapezoidal profile constraints to the provided logger backend.
     *
     * @param logger the backend logger to record the constraints.
     */
    public void logConstraints() {
      Logger.recordOutput("Sim/maxValue", maxValue.baseUnitMagnitude());
      Logger.recordOutput("Sim/maxSlew", maxSlew.baseUnitMagnitude());
      Logger.recordOutput("Sim/maxSlewSlew", maxSlewSlew.baseUnitMagnitude());
      Logger.recordOutput("Sim/constraintUnit", maxValue.unit().name());
    }

    /**
     * Calculates the next state of the trapezoidal profile given the current state, goal state, and
     * elapsed time.
     *
     * @param current the current state.
     * @param goal the goal state.
     * @param deltaTime the elapsed time for this update.
     * @return the next state as computed by the trapezoidal profile.
     */
    @SuppressWarnings("unchecked")
    public State<DIM> calculate(State<DIM> current, State<DIM> goal, Time deltaTime) {
      var internalState =
          internalProfile.calculate(
              deltaTime.baseUnitMagnitude(),
              new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                  current.value.baseUnitMagnitude(), current.slew.baseUnitMagnitude()),
              new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                  goal.value.baseUnitMagnitude(), goal.slew.baseUnitMagnitude()));

      Measure<DIM> value = (Measure<DIM>) current.value.unit().of(internalState.position);
      Velocity<DIM> slew = VelocityUnit.combine(value.unit(), Second).of(internalState.velocity);
      if (MeasureMath.abs(value).gt(maxValue)) {
        value = MeasureMath.clamp(value, maxValue);
        return new State<>(value, (Velocity<DIM>) slew.unit().zero());
      } else {
        return new State<>(value, slew);
      }
    }
  }
}
