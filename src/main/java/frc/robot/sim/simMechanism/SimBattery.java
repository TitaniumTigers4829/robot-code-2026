package frc.robot.sim.simMechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;

/** Simulates a battery in the simulation environment. */
public class SimBattery {
  private final ConcurrentHashMap<Object, Supplier<Current>> electricalAppliances =
      new ConcurrentHashMap<>();

  /**
   * Adds a custom electrical appliance to the battery. This could be a PDH or a PDP.
   *
   * @param customElectricalAppliances A supplier that provides the current draw of the appliance.
   */
  public void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
    this.electricalAppliances.put(new Object(), customElectricalAppliances);
  }

  /**
   * Adds a mechanism to the battery.
   *
   * @param simMechanism The mechanism to add.
   */
  public void addMechanism(SimMechanism simMechanism) {
    this.electricalAppliances.put(
        simMechanism, () -> simMechanism.getMotorCurrent(getBatteryVoltage()));
  }

  /**
   * Removes a mechanism from the battery.
   *
   * @param simMechanism The mechanism to remove.
   */
  public void removeMechanism(SimMechanism simMechanism) {
    this.electricalAppliances.remove(simMechanism);
  }

  /**
   * Gets the battery voltage.
   *
   * @return The battery voltage.
   */
  public Voltage getBatteryVoltage() {
    final double[] totalCurrentAmps =
        electricalAppliances.values().stream()
            .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
            .toArray();

    double batteryVoltageVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);

    return Volts.of(MathUtil.clamp(batteryVoltageVolts, 0, 12));
  }
}
