package frc.robot.sim.configs;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.math.mathutils.GearRatio;
import frc.robot.extras.util.DCMotorExt;
import frc.robot.extras.util.ProceduralStructGenerator.IgnoreStructField;
import frc.robot.sim.simMechanism.SimMechanism.Friction;
import frc.robot.sim.simMechanism.SimMechanism.HardLimits;
import frc.robot.sim.simMechanism.SimMechanism.MechanismDynamics;

public class SimMechanismConfig implements StructSerializable {
  public DCMotorExt motor;
  public MomentOfInertia rotorInertia;
  public GearRatio gearRatio;
  public Friction friction;
  @IgnoreStructField public MechanismDynamics dynamics;
  public HardLimits limits;
  public Velocity<VoltageUnit> voltageRamp;
  public double noise;

  public SimMechanismConfig(
      DCMotorExt motor,
      MomentOfInertia rotorInertia,
      GearRatio gearRatio,
      Friction friction,
      MechanismDynamics dynamics,
      HardLimits limits,
      Velocity<VoltageUnit> voltageRamp,
      double noise) {
    this.motor = motor;
    this.rotorInertia = rotorInertia;
    this.gearRatio = gearRatio;
    this.friction = friction;
    this.dynamics = dynamics;
    this.limits = limits;
    this.voltageRamp = voltageRamp;
    this.noise = noise;
  }

  public SimMechanismConfig(DCMotorExt motor) {
    this.motor = motor;
    this.rotorInertia = KilogramSquareMeters.of(0.01);
    this.gearRatio = GearRatio.reduction(1.0);
    this.friction = Friction.zero();
    this.dynamics = MechanismDynamics.zero();
    this.limits = HardLimits.unbounded();
    this.voltageRamp = Volts.of(600.0).per(Second);
    this.noise = 0.0;
  }

  public SimMechanismConfig withRotorInertia(MomentOfInertia rotorInertia) {
    this.rotorInertia = rotorInertia;
    return this;
  }

  public SimMechanismConfig withGearRatio(GearRatio gearRatio) {
    this.gearRatio = gearRatio;
    return this;
  }

  public SimMechanismConfig withFriction(Friction friction) {
    this.friction = friction;
    return this;
  }

  public SimMechanismConfig withFriction(
      Voltage staticFrictionVolts, Voltage kineticFrictionVoltage) {
    this.friction = Friction.of(motor, staticFrictionVolts, kineticFrictionVoltage);
    return this;
  }

  public SimMechanismConfig withDynamics(MechanismDynamics dynamics) {
    this.dynamics = dynamics;
    return this;
  }

  public SimMechanismConfig withLimits(HardLimits limits) {
    this.limits = limits;
    return this;
  }

  public SimMechanismConfig withVoltageRamp(Velocity<VoltageUnit> voltageRamp) {
    this.voltageRamp = voltageRamp;
    return this;
  }

  public SimMechanismConfig withNoise(double noise) {
    this.noise = noise;
    return this;
  }

  // public static final Struct<SimMechanismConfig> struct =
  // ProceduralStructGenerator.genObjectNoUnpack(SimMechanismConfig.class);
}
