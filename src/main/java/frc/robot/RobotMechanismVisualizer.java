package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Visualizes the robot mechanism using Mechanism2d for dashboard display. Shows chassis, turret,
 * shooter, and hood in a 2D stick-figure representation.
 *
 * <p>Logs to AdvantageKit under the key "Mechanism/Robot". Call {@link #update()} periodically from
 * robotPeriodic() to refresh the display.
 */
public class RobotMechanismVisualizer {

  // Mechanism2d components
  private final LoggedMechanism2d mech2d;
  private final LoggedMechanismRoot2d mechRoot;

  // Ligaments (visual elements)
  private final LoggedMechanismLigament2d chassisBase;
  private final LoggedMechanismLigament2d chassisFrontLeft;
  private final LoggedMechanismLigament2d chassisFrontRight;
  private final LoggedMechanismLigament2d chassisBackLeft;
  private final LoggedMechanismLigament2d chassisBackRight;
  private final LoggedMechanismLigament2d turretArm;
  private final LoggedMechanismLigament2d shooterWheel;
  private final LoggedMechanismLigament2d hoodAngle;

  // Simulation-only elements (only created if isSim is true)
  private LoggedMechanismLigament2d shotTrajectory;
  private LoggedMechanismLigament2d targetIndicator;

  // State
  private double turretAngleDegrees = 90.0;
  private double shooterSpeedRPM = 0.0;
  private double hoodAngleDegrees = 0.0;
  private boolean isEnabled = false;
  private final boolean isSim;

  // Constants
  private static final double CANVAS_WIDTH = 3.0;
  private static final double CANVAS_HEIGHT = 3.0;
  private static final double CHASSIS_LENGTH = 0.8;
  private static final double TURRET_LENGTH = 0.5;
  private static final double SHOOTER_LENGTH = 0.3;
  private static final double HOOD_LENGTH = 0.2;

  /**
   * Creates a new robot mechanism visualizer.
   *
   * @param isSim True if running in simulation mode (adds extra visualization elements)
   */
  public RobotMechanismVisualizer(boolean isSim) {
    this.isSim = isSim;

    // Create canvas
    mech2d = new LoggedMechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);

    // Root at bottom center
    mechRoot = mech2d.getRoot("Robot Root", CANVAS_WIDTH / 2, 0);

    // Create chassis representation
    chassisBase =
        mechRoot.append(
            new LoggedMechanismLigament2d(
                "Chassis Base", CHASSIS_LENGTH, 0, 8, new Color8Bit(Color.kGray)));

    // Swerve modules as small wheels
    chassisFrontLeft =
        chassisBase.append(
            new LoggedMechanismLigament2d(
                "Front Left Wheel", 0.15, 45, 2, new Color8Bit(Color.kBlack)));
    chassisFrontRight =
        chassisBase.append(
            new LoggedMechanismLigament2d(
                "Front Right Wheel", 0.15, -45, 2, new Color8Bit(Color.kBlack)));
    chassisBackLeft =
        chassisBase.append(
            new LoggedMechanismLigament2d(
                "Back Left Wheel", 0.15, 135, 2, new Color8Bit(Color.kBlack)));
    chassisBackRight =
        chassisBase.append(
            new LoggedMechanismLigament2d(
                "Back Right Wheel", 0.15, -135, 2, new Color8Bit(Color.kBlack)));

    // Turret arm (rotates)
    turretArm =
        chassisBase.append(
            new LoggedMechanismLigament2d(
                "Turret", TURRET_LENGTH, 90, 4, new Color8Bit(Color.kBlue)));

    // Shooter (changes color based on speed)
    shooterWheel =
        turretArm.append(
            new LoggedMechanismLigament2d(
                "Shooter", SHOOTER_LENGTH, 0, 6, new Color8Bit(Color.kRed)));

    // Hood (changes angle)
    hoodAngle =
        shooterWheel.append(
            new LoggedMechanismLigament2d("Hood", HOOD_LENGTH, 0, 3, new Color8Bit(Color.kGreen)));

    // Add simulation-only elements if in sim mode
    if (isSim) {
      // Create a separate root for simulation overlays
      LoggedMechanismRoot2d simRoot = mech2d.getRoot("Sim Overlay", 0, 0);

      shotTrajectory =
          simRoot.append(
              new LoggedMechanismLigament2d(
                  "Shot Trajectory", 1.0, 45, 1, new Color8Bit(255, 255, 255))); // Semi-transparent

      targetIndicator =
          simRoot.append(
              new LoggedMechanismLigament2d("Target", 0.1, 0, 5, new Color8Bit(Color.kYellow)));

      // Hide initially
      shotTrajectory.setLength(0);
      targetIndicator.setLength(0);
    }

    // No SmartDashboard publish – we'll log via AdvantageKit in update()
  }

  /**
   * Updates the visualization with current state and logs to AdvantageKit. Call this periodically
   * from robotPeriodic().
   */
  public void update() {
    updateTurret();
    updateShooter();
    updateHood();
    updateChassisVisuals();

    // Log the mechanism to AdvantageKit
    Logger.recordOutput("Mechanism/Robot", mech2d);
  }

  /** Updates turret angle visualization. */
  private void updateTurret() {
    turretArm.setAngle(turretAngleDegrees);
  }

  /** Updates shooter visualization based on speed. */
  private void updateShooter() {
    // Change color based on speed
    if (Math.abs(shooterSpeedRPM) > 0.1) {
      // Interpolate between red and orange based on speed
      int red = 255;
      int green = (int) Math.min(100 + (Math.abs(shooterSpeedRPM) / 100) * 155, 255);
      shooterWheel.setColor(new Color8Bit(red, green, 0));
    } else {
      shooterWheel.setColor(new Color8Bit(Color.kRed));
    }

    // Make shooter "spin" by alternating length slightly (visual effect)
    double pulse = Math.sin(System.currentTimeMillis() / 100.0) * 0.02;
    shooterWheel.setLength(SHOOTER_LENGTH + pulse);
  }

  /** Updates hood angle visualization. */
  private void updateHood() {
    hoodAngle.setAngle(hoodAngleDegrees);
  }

  /** Updates chassis visual effects (e.g., show when enabled). */
  private void updateChassisVisuals() {
    if (isEnabled) {
      chassisBase.setColor(new Color8Bit(Color.kGreen));
    } else {
      chassisBase.setColor(new Color8Bit(Color.kGray));
    }
  }

  /**
   * Sets the current turret angle.
   *
   * @param angleDegrees Turret angle in degrees
   */
  public void setTurretAngle(double angleDegrees) {
    this.turretAngleDegrees = angleDegrees;
  }

  /**
   * Sets the current shooter speed.
   *
   * @param speedRPM Shooter speed in RPM
   */
  public void setShooterSpeed(double speedRPM) {
    this.shooterSpeedRPM = speedRPM;
  }

  /**
   * Sets the current hood angle.
   *
   * @param angleDegrees Hood angle in degrees
   */
  public void setHoodAngle(double angleDegrees) {
    this.hoodAngleDegrees = angleDegrees;
  }

  /**
   * Sets the robot enabled state.
   *
   * @param enabled True if robot is enabled
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }

  /**
   * Shows a predicted shot trajectory (simulation only).
   *
   * @param angle Angle of the shot in degrees (field-relative)
   * @param distance Distance to target in meters (will be scaled for visualization)
   */
  public void showShotTrajectory(double angle, double distance) {
    if (!isSim || shotTrajectory == null) return;

    shotTrajectory.setAngle(angle);
    // Scale distance to fit on canvas (max 3 meters)
    double scaledLength = Math.min(distance / 5.0, 2.0);
    shotTrajectory.setLength(scaledLength);

    // Position target at the end of trajectory (simplified)
    if (targetIndicator != null) {
      targetIndicator.setAngle(angle);
      targetIndicator.setLength(scaledLength + 0.1);
    }
  }

  /** Hides the shot trajectory (simulation only). */
  public void hideShotTrajectory() {
    if (!isSim || shotTrajectory == null) return;
    shotTrajectory.setLength(0);
    if (targetIndicator != null) {
      targetIndicator.setLength(0);
    }
  }

  /**
   * Gets the underlying Mechanism2d object for advanced use.
   *
   * @return The Mechanism2d instance
   */
  public LoggedMechanism2d getMechanism2d() {
    return mech2d;
  }
}
