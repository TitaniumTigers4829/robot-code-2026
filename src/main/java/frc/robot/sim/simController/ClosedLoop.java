package frc.robot.sim.simController;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.sim.simController.UnitSafeControl.ArmFeedforward;
import frc.robot.sim.simController.UnitSafeControl.ElevatorFeedforward;
import frc.robot.sim.simController.UnitSafeControl.Feedforward;
import frc.robot.sim.simController.UnitSafeControl.FlywheelFeedforward;
import frc.robot.sim.simController.UnitSafeControl.PIDFeedback;
import frc.robot.sim.simController.UnitSafeControl.TrapezoidProfile;
import frc.robot.sim.simController.UnitSafeControl.TrapezoidProfile.State;
import java.util.Optional;
import java.util.function.Function;

/**
 * Represents a closed-loop controller that combines a PID feedback controller and a feedforward
 * controller, optionally using a trapezoidal motion profile for smoothing.
 *
 * <p>This generic controller supports various unit types for output, input, and input dimensions,
 * and can be specialized for voltage or current control for position, velocity, or distance.
 *
 * @param <OUTPUT> the unit type of the controller output (e.g., VoltageUnit or CurrentUnit)
 * @param <INPUT> the unit type of the feedback controller input (e.g., AngleUnit,
 *     AngularVelocityUnit, or DistanceUnit)
 * @param <INPUT_DIMENSION> the unit type representing the dimensional measurement (e.g., AngleUnit
 *     or DistanceUnit)
 */
public class ClosedLoop<OUTPUT extends Unit, INPUT extends Unit, INPUT_DIMENSION extends Unit> {
  private final PIDFeedback<OUTPUT, INPUT> feedback;
  private final Feedforward<OUTPUT, INPUT_DIMENSION> feedforward;
  private final Optional<TrapezoidProfile<INPUT>> optTrapezoidProfile;
  private final boolean useFeedbackSign;
  private final Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance;

  /**
   * Constructs a ClosedLoop controller without a custom angle-to-distance conversion function.
   *
   * @param feedback the PID feedback controller
   * @param feedforward the feedforward controller
   * @param trapezoidProfile an optional trapezoidal motion profile for the input
   * @param useFeedbackSign flag indicating whether to use the sign of the feedback output for
   *     feedforward calculation
   */
  private ClosedLoop(
      PIDFeedback<OUTPUT, INPUT> feedback,
      Feedforward<OUTPUT, INPUT_DIMENSION> feedforward,
      Optional<TrapezoidProfile<INPUT>> trapezoidProfile,
      boolean useFeedbackSign) {
    this.feedback = feedback;
    this.feedforward = feedforward;
    this.optTrapezoidProfile = trapezoidProfile;
    this.useFeedbackSign = useFeedbackSign;
    // Default conversion returns zero distance.
    this.angleToDistance = angle -> Meters.zero();
  }

  /**
   * Constructs a ClosedLoop controller with a custom angle-to-distance conversion function.
   *
   * @param feedback the PID feedback controller
   * @param feedforward the feedforward controller
   * @param trapezoidProfile an optional trapezoidal motion profile for the input
   * @param useFeedbackSign flag indicating whether to use the sign of the feedback output for
   *     feedforward calculation
   * @param angleToDistance a function to convert an angle measurement to a distance measurement
   */
  private ClosedLoop(
      PIDFeedback<OUTPUT, INPUT> feedback,
      Feedforward<OUTPUT, INPUT_DIMENSION> feedforward,
      Optional<TrapezoidProfile<INPUT>> trapezoidProfile,
      boolean useFeedbackSign,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    this.feedback = feedback;
    this.feedforward = feedforward;
    this.optTrapezoidProfile = trapezoidProfile;
    this.useFeedbackSign = useFeedbackSign;
    this.angleToDistance = angleToDistance;
  }

  // ===========================================================================
  // Factory methods for different controller types.
  // ===========================================================================

  /**
   * Creates a ClosedLoop controller for voltage control using angle feedback.
   *
   * @param feedback the PID feedback controller for angle
   * @param feedforward the feedforward controller for angle
   * @param trapezoidProfile the trapezoidal profile to smooth the angle trajectory
   * @return a new ClosedLoop instance configured for voltage-angle control
   */
  public static ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  /**
   * Creates a ClosedLoop controller for voltage control using angle feedback without a trapezoidal
   * profile.
   *
   * @param feedback the PID feedback controller for angle
   * @param feedforward the feedforward controller for angle
   * @param useFeedbackSign flag indicating whether to use the sign of the feedback output for
   *     feedforward calculation
   * @return a new ClosedLoop instance configured for voltage-angle control
   */
  public static ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> forVoltageAngle(
      PIDFeedback<VoltageUnit, AngleUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward,
      boolean useFeedbackSign) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), useFeedbackSign);
  }

  /**
   * Creates a ClosedLoop controller for voltage control using angular velocity feedback.
   *
   * @param feedback the PID feedback controller for angular velocity
   * @param feedforward the feedforward controller for angle
   * @param trapezoidProfile the trapezoidal profile to smooth the angular velocity trajectory
   * @return a new ClosedLoop instance configured for voltage-angular velocity control
   */
  public static ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  /**
   * Creates a ClosedLoop controller for voltage control using angular velocity feedback without a
   * trapezoidal profile.
   *
   * @param feedback the PID feedback controller for angular velocity
   * @param feedforward the feedforward controller for angle
   * @return a new ClosedLoop instance configured for voltage-angular velocity control
   */
  public static ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> forVoltageAngularVelocity(
      PIDFeedback<VoltageUnit, AngularVelocityUnit> feedback,
      Feedforward<VoltageUnit, AngleUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false);
  }

  /**
   * Creates a ClosedLoop controller for current control using angle feedback.
   *
   * @param feedback the PID feedback controller for angle
   * @param feedforward the feedforward controller for angle
   * @param trapezoidProfile the trapezoidal profile to smooth the angle trajectory
   * @return a new ClosedLoop instance configured for current-angle control
   */
  public static ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngleUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  /**
   * Creates a ClosedLoop controller for current control using angle feedback without a trapezoidal
   * profile.
   *
   * @param feedback the PID feedback controller for angle
   * @param feedforward the feedforward controller for angle
   * @param useFeedbackSign flag indicating whether to use the sign of the feedback output for
   *     feedforward calculation
   * @return a new ClosedLoop instance configured for current-angle control
   */
  public static ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> forCurrentAngle(
      PIDFeedback<CurrentUnit, AngleUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward,
      boolean useFeedbackSign) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), useFeedbackSign);
  }

  /**
   * Creates a ClosedLoop controller for current control using angular velocity feedback.
   *
   * @param feedback the PID feedback controller for angular velocity
   * @param feedforward the feedforward controller for angle
   * @param trapezoidProfile the trapezoidal profile to smooth the angular velocity trajectory
   * @return a new ClosedLoop instance configured for current-angular velocity control
   */
  public static ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward,
      TrapezoidProfile<AngularVelocityUnit> trapezoidProfile) {
    return new ClosedLoop<>(feedback, feedforward, Optional.of(trapezoidProfile), false);
  }

  /**
   * Creates a ClosedLoop controller for current control using angular velocity feedback without a
   * trapezoidal profile.
   *
   * @param feedback the PID feedback controller for angular velocity
   * @param feedforward the feedforward controller for angle
   * @return a new ClosedLoop instance configured for current-angular velocity control
   */
  public static ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> forCurrentAngularVelocity(
      PIDFeedback<CurrentUnit, AngularVelocityUnit> feedback,
      Feedforward<CurrentUnit, AngleUnit> feedforward) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false);
  }

  /**
   * Creates a ClosedLoop controller for voltage control using distance feedback.
   *
   * @param feedback the PID feedback controller for distance
   * @param feedforward the feedforward controller for distance
   * @param trapezoidProfile the trapezoidal profile to smooth the distance trajectory
   * @param angleToDistance a function to convert an angle measurement to a distance measurement
   * @return a new ClosedLoop instance configured for voltage-distance control
   */
  public static ClosedLoop<VoltageUnit, DistanceUnit, DistanceUnit> forVoltageDistance(
      PIDFeedback<VoltageUnit, DistanceUnit> feedback,
      Feedforward<VoltageUnit, DistanceUnit> feedforward,
      TrapezoidProfile<DistanceUnit> trapezoidProfile,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.of(trapezoidProfile), false, angleToDistance);
  }

  /**
   * Creates a ClosedLoop controller for voltage control using distance feedback without a
   * trapezoidal profile.
   *
   * @param feedback the PID feedback controller for distance
   * @param feedforward the feedforward controller for distance
   * @param angleToDistance a function to convert an angle measurement to a distance measurement
   * @return a new ClosedLoop instance configured for voltage-distance control
   */
  public static ClosedLoop<VoltageUnit, DistanceUnit, DistanceUnit> forVoltageDistance(
      PIDFeedback<VoltageUnit, DistanceUnit> feedback,
      Feedforward<VoltageUnit, DistanceUnit> feedforward,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(feedback, feedforward, Optional.empty(), false, angleToDistance);
  }

  /**
   * Creates a ClosedLoop controller for current control using distance feedback.
   *
   * @param feedback the PID feedback controller for distance
   * @param feedforward the feedforward controller for distance
   * @param trapezoidProfile the trapezoidal profile to smooth the distance trajectory
   * @param angleToDistance a function to convert an angle measurement to a distance measurement
   * @return a new ClosedLoop instance configured for current-distance control
   */
  public static ClosedLoop<CurrentUnit, DistanceUnit, DistanceUnit> forCurrentDistance(
      PIDFeedback<CurrentUnit, DistanceUnit> feedback,
      Feedforward<CurrentUnit, DistanceUnit> feedforward,
      TrapezoidProfile<DistanceUnit> trapezoidProfile,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.of(trapezoidProfile), false, angleToDistance);
  }

  /**
   * Creates a ClosedLoop controller for current control using distance feedback without a
   * trapezoidal profile.
   *
   * @param feedback the PID feedback controller for distance
   * @param feedforward the feedforward controller for distance
   * @param useFeedbackSign flag indicating whether to use the sign of the feedback output for
   *     feedforward calculation
   * @param angleToDistance a function to convert an angle measurement to a distance measurement
   * @return a new ClosedLoop instance configured for current-distance control
   */
  public static ClosedLoop<CurrentUnit, DistanceUnit, DistanceUnit> forCurrentDistance(
      PIDFeedback<CurrentUnit, DistanceUnit> feedback,
      Feedforward<CurrentUnit, DistanceUnit> feedforward,
      boolean useFeedbackSign,
      Function<Measure<AngleUnit>, Measure<DistanceUnit>> angleToDistance) {
    return new ClosedLoop<>(
        feedback, feedforward, Optional.empty(), useFeedbackSign, angleToDistance);
  }

  /**
   * Checks whether the feedback controller is using a velocity unit.
   *
   * @return {@code true} if the input unit of the feedback is a PerUnit (velocity), {@code false}
   *     otherwise.
   */
  public boolean isVelocity() {
    return feedback.getInputUnit() instanceof PerUnit;
  }

  /**
   * Converts a state from angle units to distance units using the {@code angleToDistance} function.
   *
   * @param state the state in angle units
   * @return the corresponding state in distance units with adjusted velocity
   */
  private State<DistanceUnit> angleToDistanceState(State<AngleUnit> state) {
    final VelocityUnit<DistanceUnit> velocityUnit = VelocityUnit.combine(Meters, Seconds);
    final double ratio = angleToDistance.apply(Radians.of(1)).baseUnitMagnitude();
    return new State<>(
        angleToDistance.apply(state.value()),
        velocityUnit.of(state.slew().baseUnitMagnitude() * ratio));
  }

  /**
   * Converts a state from angular velocity units to linear velocity units using the {@code
   * angleToDistance} conversion.
   *
   * @param state the state in angular velocity units
   * @return the corresponding state in linear velocity units
   */
  private State<LinearVelocityUnit> angularVelocityToLinearVelocityState(
      State<AngularVelocityUnit> state) {
    final double ratio = angleToDistance.apply(Radians.of(1)).baseUnitMagnitude();
    final VelocityUnit<LinearVelocityUnit> velocityUnit =
        VelocityUnit.combine(MetersPerSecond, Seconds);
    return new State<>(
        MetersPerSecond.of(state.value().baseUnitMagnitude() * ratio),
        velocityUnit.of(state.slew().baseUnitMagnitude() * ratio));
  }

  /**
   * Runs the closed-loop controller for position control.
   *
   * <p>This method takes an angle measurement and, if the feedback input unit is a DistanceUnit,
   * converts the position and state to distance before running the controller.
   *
   * @param position the current position as an angle
   * @param state the current state (position and velocity) in angle units
   * @param goal the desired goal state in angle units
   * @param dt the elapsed time step
   * @return the computed controller output as a measure in the output unit
   * @throws UnsupportedOperationException if the feedback input unit is velocity-based
   */
  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> runPosition(
      Angle position, State<AngleUnit> state, State<AngleUnit> goal, Time dt) {
    if (isVelocity()) {
      throw new UnsupportedOperationException("Velocity not supported for `runAngle`");
    }
    if (feedback.getInputUnit() instanceof DistanceUnit) {
      return run(
          (Measure<INPUT_DIMENSION>) angleToDistance.apply(position),
          (State<INPUT>) angleToDistanceState(state),
          (State<INPUT>) angleToDistanceState(goal),
          dt);
    } else {
      return run(
          (Measure<INPUT_DIMENSION>) position, (State<INPUT>) state, (State<INPUT>) goal, dt);
    }
  }

  /**
   * Runs the closed-loop controller for velocity control.
   *
   * <p>This method takes an angle measurement and, if the feedback input unit is a DistanceUnit,
   * converts the position and state to linear velocity before running the controller.
   *
   * @param position the current position as an angle
   * @param state the current state (velocity and acceleration) in angular velocity units
   * @param goal the desired goal state in angular velocity units
   * @param dt the elapsed time step
   * @return the computed controller output as a measure in the output unit
   * @throws UnsupportedOperationException if the feedback input unit is not velocity-based
   */
  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> runVelocity(
      Angle position, State<AngularVelocityUnit> state, State<AngularVelocityUnit> goal, Time dt) {
    if (!isVelocity()) {
      throw new UnsupportedOperationException("Position not supported for `runVelocity`");
    }
    if (feedback.getInputUnit() instanceof DistanceUnit) {
      return run(
          (Measure<INPUT_DIMENSION>) angleToDistance.apply(position),
          (State<INPUT>) angularVelocityToLinearVelocityState(state),
          (State<INPUT>) angularVelocityToLinearVelocityState(goal),
          dt);
    } else {
      return run(
          (Measure<INPUT_DIMENSION>) position, (State<INPUT>) state, (State<INPUT>) goal, dt);
    }
  }

  /**
   * Runs the closed-loop controller using the provided position, state, and goal.
   *
   * <p>If a trapezoidal motion profile is provided, the controller computes a step along the
   * profile and then combines the PID feedback output with the appropriate feedforward output.
   * Otherwise, it directly combines the feedback and feedforward outputs.
   *
   * @param position the current position (or converted distance) measurement
   * @param state the current state (position and velocity) in the input unit
   * @param goal the desired goal state in the input unit
   * @param dt the elapsed time step
   * @return the combined controller output as a measure in the output unit
   * @throws UnsupportedOperationException if the feedforward type is not supported
   */
  @SuppressWarnings("unchecked")
  public Measure<OUTPUT> run(
      Measure<INPUT_DIMENSION> position, State<INPUT> state, State<INPUT> goal, Time dt) {
    boolean isVelocity = feedback.getInputUnit() instanceof PerUnit;
    if (optTrapezoidProfile.isPresent()) {
      var trapezoidProfile = optTrapezoidProfile.get();
      State<INPUT> step = trapezoidProfile.calculate(state, goal, dt);
      Measure<OUTPUT> feedforwardOutput = (Measure<OUTPUT>) feedback.getOutputUnit().zero();
      if (feedforward.getClass().equals(FlywheelFeedforward.class)) {
        var flywheelFF = (FlywheelFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? flywheelFF.calculate(
                    RadiansPerSecond.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : flywheelFF.calculate(RadiansPerSecond.of(step.slew().baseUnitMagnitude()));
      } else if (feedforward.getClass().equals(ElevatorFeedforward.class)) {
        var elevatorFF = (ElevatorFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? elevatorFF.calculate(
                    MetersPerSecond.of(step.value().baseUnitMagnitude()),
                    MetersPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : elevatorFF.calculate(MetersPerSecond.of(step.slew().baseUnitMagnitude()));
      } else if (feedforward.getClass().equals(ArmFeedforward.class)) {
        var armFF = (ArmFeedforward<OUTPUT>) feedforward;
        feedforwardOutput =
            isVelocity
                ? armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(step.slew().baseUnitMagnitude()))
                : armFF.calculate(
                    Radians.of(step.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(step.slew().baseUnitMagnitude()));
      } else {
        throw new UnsupportedOperationException("Feedforward type not supported");
      }
      var fbResult = feedback.calculate(state.value(), step.value());
      Measure<OUTPUT> feedbackOutput = fbResult.getFirst();
      return feedbackOutput.plus(feedforwardOutput);
    } else {
      var fbResult = feedback.calculate(state.value(), goal.value());
      Measure<OUTPUT> feedbackOutput = fbResult.getFirst();
      Measure<OUTPUT> feedforwardOutput = (Measure<OUTPUT>) feedback.getOutputUnit().zero();
      double velocitySign =
          useFeedbackSign
              ? 0.00001 * Math.signum(feedbackOutput.baseUnitMagnitude())
              : 0.00001
                  * Math.signum(
                      goal.value().baseUnitMagnitude() - state.value().baseUnitMagnitude());
      if (feedforward.getClass().equals(FlywheelFeedforward.class)) {
        var flywheelFF = (FlywheelFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                flywheelFF.calculate(RadiansPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                flywheelFF.calculate(
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else if (feedforward.getClass().equals(ElevatorFeedforward.class)) {
        var elevatorFF = (ElevatorFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(MetersPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                elevatorFF.calculate(
                    MetersPerSecond.of(goal.value().baseUnitMagnitude()),
                    MetersPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else if (feedforward.getClass().equals(ArmFeedforward.class)) {
        var armFF = (ArmFeedforward<OUTPUT>) feedforward;
        if (isVelocity) {
          if (goal.slew() == null) {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()));
          } else {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(position.baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecondPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        } else {
          if (goal.slew() == null) {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(velocitySign));
          } else {
            feedforwardOutput =
                armFF.calculate(
                    Radians.of(goal.value().baseUnitMagnitude()),
                    RadiansPerSecond.of(goal.slew().baseUnitMagnitude()));
          }
        }
      } else {
        throw new UnsupportedOperationException("Feedforward type not supported");
      }
      return feedbackOutput.plus(feedforwardOutput);
    }
  }
}
