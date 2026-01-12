package frc.robot.sim.simMechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.extras.math.mathutils.GearRatio;
import frc.robot.extras.math.mathutils.MeasureMath;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.extras.util.ProceduralStructGenerator;
import frc.robot.sim.simController.SimMotorController;
import frc.robot.sim.simController.SimMotorController.ControllerOutput;
import frc.robot.sim.simField.SimArena.SimEnvTiming;
import java.util.Random;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/** This class is used to simulate a mechanism in a physics simulation. */
public class SimMechanism {
  private static final double kMotorEfficiency = 0.85;

  private static final Random RAND = new Random();

  private static final AngleUnit Rad = Radians;
  private static final AngularVelocityUnit RadPS = RadiansPerSecond;
  private static final AngularAccelerationUnit RadPS2 = RadiansPerSecondPerSecond;

  /**
   * An interface to define systemic impacts on a mechanism.
   *
   * <p>This could be the force of gravity, a spring, inertial load due to chassis acceleration,
   * etc.
   *
   * <p>It is sound behavior for this to capture outputs of other mechanisms but be aware the order
   * of calculations between mechanisms is not guaranteed. For example, if an inverse pendulum
   * mechanism could have extra load depending on the drive mechanism's acceleration, the drive
   * mechanism should be calculated first in the same time slot but this can not be promised.
   */
  public interface MechanismDynamics {
    default Torque environment(MechanismState state) {
      return NewtonMeters.zero();
    }

    /**
     * Gets the extra inertia acting on the mechanism.
     *
     * @return the extra inertia acting on the mechanism
     */
    default MomentOfInertia extraInertia() {
      return KilogramSquareMeters.zero();
    }

    static MechanismDynamics of(Torque environment) {
      return new MechanismDynamics() {
        @Override
        public Torque environment(MechanismState state) {
          return environment;
        }

        @Override
        public MomentOfInertia extraInertia() {
          return KilogramSquareMeters.zero();
        }
      };
    }

    static MechanismDynamics zero() {
      return MechanismDynamics.of(NewtonMeters.zero());
    }
  }

  /**
   * Defines the friction of a mechanism. The only way to obtain this value is to empirically
   * measure it. This record allows defining separate static and kinetic friction values but it is
   * perfectly valid to use the same value for both.
   *
   * <p>Creating a friction using {@code kS} is a valid approach and can be done like so:
   *
   * <pre><code>
   * //could be any motor
   * DCMotor motor = DCMotor.getFalcon500(1);
   * Friction.of(motor, Volts.of(kMechanism.kS));
   * </code></pre>
   */
  public record Friction(Torque staticFriction, Torque kineticFriction)
      implements StructSerializable {
    public static Friction of(Torque staticFriction, Torque kineticFriction) {
      return new Friction(staticFriction, kineticFriction);
    }

    /**
     * Constructs a {@link Friction} object with the same value for both static and kinetic
     * friction.
     *
     * @param universalFriction the value to use for both static and kinetic friction
     * @return a {@link Friction} object with the same value for both static and kinetic friction
     */
    public static Friction of(Torque universalFriction) {
      return new Friction(universalFriction, universalFriction);
    }

    /**
     * Constructs a {@link Friction} object using a {@link DCMotor} object and voltage friction
     * constants.
     *
     * @param motor the motor to use for the friction calculation
     * @param staticVoltage the voltage to use for the static friction calculation
     * @param kineticVoltage the voltage to use for the kinetic friction calculation
     * @return a {@link Friction} object using the given motor and voltage constants
     */
    public static Friction of(DCMotor motor, Voltage staticVoltage, Voltage kineticVoltage) {
      Torque staticTorque =
          NewtonMeters.of(motor.getTorque(motor.getCurrent(0.0, staticVoltage.in(Volts))));
      Torque kineticTorque =
          NewtonMeters.of(motor.getTorque(motor.getCurrent(0.0, kineticVoltage.in(Volts))));
      return new Friction(staticTorque, kineticTorque);
    }

    public static Friction of(DCMotor motor, Voltage universalVoltage) {
      return of(motor, universalVoltage, universalVoltage);
    }

    public static Friction zero() {
      return new Friction(NewtonMeters.zero(), NewtonMeters.zero());
    }

    public static final Struct<Friction> struct =
        ProceduralStructGenerator.genRecord(Friction.class);
  }

  /**
   * Defines the mechanical positional limits of the mechanism.
   *
   * @see #unbounded() HardLimits.unbounded() for a mechanism with no limits
   */
  public record HardLimits(Angle minAngle, Angle maxAngle) implements StructSerializable {
    public static HardLimits of(Angle minAngle, Angle maxAngle) {
      return new HardLimits(minAngle, maxAngle);
    }

    /**
     * Constructs a {@link HardLimits} object with no limits.
     *
     * @return a {@link HardLimits} object with no limits
     */
    public static HardLimits unbounded() {
      return new HardLimits(Rad.of(Double.NEGATIVE_INFINITY), Rad.of(Double.POSITIVE_INFINITY));
    }

    public static final Struct<HardLimits> struct =
        ProceduralStructGenerator.genRecord(HardLimits.class);
  }

  /** Defines the state of a mechanism. */
  public record MechanismState(
      Angle position, AngularVelocity velocity, AngularAcceleration acceleration)
      implements StructSerializable {

    public static MechanismState of(
        Angle angle, AngularVelocity velocity, AngularAcceleration acceleration) {
      return new MechanismState(angle, velocity, acceleration);
    }

    public static MechanismState zero() {
      return new MechanismState(Rad.zero(), RadPS.zero(), RadPS2.zero());
    }

    public MechanismState times(double scalar) {
      return new MechanismState(
          position.times(scalar), velocity.times(scalar), acceleration.times(scalar));
    }

    public MechanismState div(double scalar) {
      return times(1.0 / scalar);
    }

    public static final Struct<MechanismState> struct =
        ProceduralStructGenerator.genRecord(MechanismState.class);
  }

  /** Defines some variables of a mechanism in the current time step. */
  public record MechanismVariables(
      Torque torque, Voltage statorVoltage, Voltage supplyVoltage, Current statorCurrent)
      implements StructSerializable {

    public Current supplyCurrent() {
      // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10;
      return statorCurrent.times(statorVoltage.div(supplyVoltage)).div(kMotorEfficiency);
    }

    public static MechanismVariables of(
        Torque torque, Voltage voltage, Voltage supplyVoltage, Current statorCurrent) {
      return new MechanismVariables(torque, voltage, supplyVoltage, statorCurrent);
    }

    public static MechanismVariables zero() {
      return new MechanismVariables(NewtonMeters.zero(), Volts.zero(), Volts.zero(), Amps.zero());
    }

    public static final Struct<MechanismVariables> struct =
        ProceduralStructGenerator.genRecord(MechanismVariables.class);
  }

  // Create variables to store the mechanism's properties
  private final String name;
  private final MechanismDynamics dynamics;
  private final Friction friction;
  private final GearRatio gearRatio;
  private final DCMotorExt motor;
  private final SimMotorController controller;
  private final SimEnvTiming timing;
  private final MomentOfInertia rotorInertia;
  private final HardLimits limits;
  private final double noise;

  /**
   * A lock to ensure that the mechanism state and variables are not modified while they are being
   * read
   */
  private final ReentrantReadWriteLock ioLock = new ReentrantReadWriteLock();

  /**
   * The current state of the mechanism. This is the state of the mechanism at the current time
   * step.
   */
  private MechanismState state = MechanismState.zero();

  /**
   * The current variables of the mechanism. This is the variables of the mechanism at the current
   * time step.
   */
  private MechanismVariables variables = MechanismVariables.zero();

  /** Constructs a new SimMechanism. */
  public SimMechanism(
      String name,
      DCMotorExt motor,
      SimMotorController controller,
      MomentOfInertia rotorInertia,
      GearRatio gearRatio,
      Friction friction,
      MechanismDynamics dynamics,
      HardLimits limits,
      double noise,
      SimEnvTiming timing) {
    this.dynamics = dynamics;
    this.friction = friction;
    this.gearRatio = gearRatio;
    this.motor = motor;
    this.controller = controller;
    this.timing = timing;
    this.rotorInertia = rotorInertia;
    this.name = name;
    this.limits = limits;
    this.noise = noise;

    // Configure the controllers motor model to the mechanism's motor
    controller.configureMotorModel(this.motor);
  }

  /**
   * Gets the current state of the mechanism.
   *
   * @return the current state of the mechanism
   */
  public MechanismState state() {
    try {
      ioLock.readLock().lock();
      return state;
    } finally {
      ioLock.readLock().unlock();
    }
  }

  /**
   * Gets the current state of the motor driving the mechanism.
   *
   * @return the current state of the mechanism
   */
  public MechanismState motorState() {
    return state().times(gearRatio.getReduction());
  }

  /**
   * Sets the current state of the mechanism.
   *
   * @param state the new state of the mechanism
   */
  public void setState(MechanismState state) {
    try {
      ioLock.writeLock().lock();
      this.state = state;
    } finally {
      ioLock.writeLock().unlock();
    }
  }

  /** Gets the current variables of the mechanism. */
  public MechanismVariables variables() {
    try {
      ioLock.readLock().lock();
      return variables;
    } finally {
      ioLock.readLock().unlock();
    }
  }

  /** Gets the current state of the motor driving the mechanism. */
  public MechanismVariables motorVariables() {
    var v = variables();
    return MechanismVariables.of(
        v.torque.div(gearRatio.getReduction()), v.statorVoltage, v.supplyVoltage, v.statorCurrent);
  }

  /**
   * Gets the name of the mechanism.
   *
   * @return the name of the mechanism
   */
  public String name() {
    return name;
  }

  /**
   * Gets the torque applied by the motor when "braking" is enabled.
   *
   * <p>This value is before any gear reduction is applied.
   *
   * @return the torque applied by the motor when "braking" is enabled.
   */
  protected Torque getMotorBrakingTorque() {
    if (!controller.brakeEnabled()) {
      return NewtonMeters.zero();
    }
    Current current = motor.getCurrent(motorState().velocity(), Volts.zero());
    Torque torque = motor.getTorque(current);
    return torque.times(gearRatio.getReduction());
  }

  /**
   * Applies friction and braking to the output of the mechanism. This will reduce the torque
   * applied to the mechanism.
   *
   * @param torque the torque to apply to the mechanism
   * @return the torque after friction has been applied
   */
  protected Torque calculateResistanceTorque(
      Torque motorOutput, Torque environment, MomentOfInertia inertia) {
    AngularVelocity velocity = state().velocity();
    boolean isMoving = MeasureMath.abs(velocity).gt(RadPS.zero());
    Torque frictionTorque = isMoving ? friction.kineticFriction() : friction.staticFriction();
    Torque brakingTorque = NewtonMeters.zero();
    if (MeasureMath.abs(motorOutput).lt(NewtonMeters.of(0.01))) {
      brakingTorque = getMotorBrakingTorque();
    }
    // ensure that friction/braking opposes the motion
    Torque antiTorque =
        frictionTorque
            .plus(brakingTorque)
            .times(-MeasureMath.signum(velocity) * gearRatio.getReduction());

    final AngularAcceleration unburdenedAccel =
        MeasureMath.div(motorOutput.plus(environment), inertia);
    final AngularAcceleration burdenAccel = MeasureMath.div(antiTorque, inertia);
    final AngularAcceleration imposedAccel = unburdenedAccel.plus(burdenAccel);
    // the goal of anti torque is to reduce the velocity to 0
    // so we need to ensure that the anti torque is not so large that it causes the mechanism to
    // reverse

    final AngularAcceleration accelNeededToStop = velocity.times(-1.0).div(timing.dt());
    final Torque torqueNeededToStop = MeasureMath.times(accelNeededToStop, inertia);

    if (accelNeededToStop.lt(RadPS2.zero())) {
      // accel needed to stop is negative
      if (imposedAccel.lt(accelNeededToStop)) {
        // the imposed accel has a greater magnitude in the same sign
        // as the accel needed to stop, this will cause the mechanism
        // to reverse
        return torqueNeededToStop;
      } else {
        // the imposed accel has a lesser magnitude in the same sign
        // as the accel needed to stop, this will cannot cause the mechanism
        // to reverse
        return antiTorque;
      }
    } else {
      // accel needed to stop is positive
      if (imposedAccel.gt(accelNeededToStop)) {
        // the imposed accel has a greater magnitude in the same sign
        // as the accel needed to stop, this will cause the mechanism
        // to reverse
        return torqueNeededToStop;
      } else {
        // the imposed accel has a lesser magnitude in the same sign
        // as the accel needed to stop, this will cannot cause the mechanism
        // to reverse
        return antiTorque;
      }
    }
  }

  /**
   * Gets the current of the motor.
   *
   * @param supplyVoltage the supply voltage of the motor.
   * @return the current of the motor.
   */
  protected Current getMotorCurrent(Voltage supplyVoltage) {
    ControllerOutput co = controller.run(timing.dt(), supplyVoltage, motorState());
    if (DriverStation.isDisabled()) {
      return Amps.zero();
    }
    if (co instanceof ControllerOutput.VoltageOutput vo) {
      Voltage voltage = vo.voltage();
      if (voltage.isNear(Volts.zero(), Volts.of(0.01))) {
        return Amps.zero();
      }
      return motor.getCurrent(motorState().velocity(), voltage);
    } else if (co instanceof ControllerOutput.CurrentOutput io) {
      return io.current();
    } else {
      return Amps.zero();
    }
  }

  /**
   * Updates the state of the mechanism.
   *
   * @param supplyVoltage the supply voltage of the motor.
   */
  public void update(final Voltage supplyVoltage) {
    final Time dt = timing.dt();

    final MomentOfInertia inertia = rotorInertia.plus(dynamics.extraInertia());

    // calculate the torque acting on the mechanism
    final Current motorCurrent = getMotorCurrent(supplyVoltage);
    final Torque motorTorque = motor.getTorque(motorCurrent);
    final Torque mechanismTorque = motorTorque.times(gearRatio.getReduction());
    final Torque environment = dynamics.environment(state());
    final Torque antiTorque = calculateResistanceTorque(mechanismTorque, environment, inertia);
    final Torque outputTorque = mechanismTorque.plus(antiTorque);

    // calculate the displacement, velocity, and acceleration of the mechanism
    final AngularAcceleration acceleration =
        MeasureMath.div(outputTorque, inertia).times(1.0 + (noise * RAND.nextGaussian()));
    final AngularVelocity velocity =
        MeasureMath.nudgeZero(
            state().velocity().plus(acceleration.times(dt)), RotationsPerSecond.of(0.001));
    final Angle angle = state().position().plus(velocity.times(dt));

    try {
      ioLock.writeLock().lock();
      // apply hard limits
      // then update the state accordingly
      if (angle.lt(limits.minAngle())) {
        state = MechanismState.of(limits.minAngle(), RadPS.zero(), RadPS2.zero());
      } else if (angle.gt(limits.maxAngle())) {
        state = MechanismState.of(limits.maxAngle(), RadPS.zero(), RadPS2.zero());
      } else {
        state = MechanismState.of(angle, velocity, acceleration);
      }
      // capture the "variables"
      variables =
          MechanismVariables.of(
              mechanismTorque,
              motor.getVoltage(motorTorque, motorState().velocity()),
              supplyVoltage,
              motorCurrent);
    } finally {
      ioLock.writeLock().unlock();
    }
  }
}
