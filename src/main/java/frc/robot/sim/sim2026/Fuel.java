package frc.robot.sim.sim2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.math.forces.ProjectileUtil.ProjectileDynamics;
import frc.robot.extras.math.forces.Velocity2d;
import frc.robot.extras.math.forces.Velocity3d;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimGamePiece;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.geometry.Geometry;

/**
 * Represents a FUEL game piece in the 2026 REBUILT game.
 *
 * <p>FUEL is a 5.91" diameter foam ball that can be:
 *
 * <ul>
 *   <li>Scored in the HUB (worth 1 point when HUB is active)
 *   <li>Staged in DEPOTs and OUTPOST CHUTEs
 *   <li>Pre-loaded in robots (up to 8 per robot)
 *   <li>Thrown by human players
 * </ul>
 */
public class Fuel extends SimGamePiece {

  // Mass in kg (converted from 0.448–0.5 lbs)
  private static final double FUEL_MASS_KG = ((0.448 + 0.5) / 2.0) * 0.453592;

  // Diameter/radius in meters
  private static final double FUEL_DIAMETER_M = FieldConstants.FUEL_DIAMETER_METERS;
  private static final double FUEL_RADIUS_M = FUEL_DIAMETER_M / 2.0;

  // ---- Alliance tracking ----
  // Set at intake time (before the piece is shot) so we can determine which
  // alliance scored even after the piece enters LIMBO — where pose() returns
  // (-1, -1, -1) and is useless for alliance detection.
  // Defaults to false (red) as a safe fallback; always call setScoringAlliance()
  // when a robot intakes this piece.
  private boolean scoringAllianceIsBlue = false;

  // ---- Newly-scored transition detection ----
  // Tracks whether this piece was already in LIMBO on the previous call to
  // isNewlyScored(), so we fire the flag exactly once per LIMBO transition
  // rather than every tick while the piece remains scored.
  private boolean wasScored = false;
  private boolean newlyScoredFlag = false;

  public Fuel(SimArena arena) {
    super(createFuelVariant(), arena);
  }

  public Fuel(SimArena arena, Translation2d position) {
    super(createFuelVariant(), arena);
    this.place(position);
  }

  public Fuel(SimArena arena, Translation2d initialPosition, Velocity2d initialVelocity) {
    super(createFuelVariant(), arena);
    this.slide(initialPosition, initialVelocity);
  }

  public Fuel(
      SimArena arena,
      Translation3d initialPose,
      Velocity3d initialVelocity,
      ProjectileDynamics dynamics) {
    super(createFuelVariant(), arena);
    this.launch(new Pose3d(initialPose, new Rotation3d()), initialVelocity, dynamics);
  }

  // -------------------------------------------------------------------------
  // Variant / target setup
  // -------------------------------------------------------------------------

  private static GamePieceVariant createFuelVariant() {
    return new GamePieceVariant(
        "FUEL_2026",
        FUEL_DIAMETER_M,
        FUEL_MASS_KG,
        Geometry.createCircle(FUEL_RADIUS_M),
        createFuelTargets(),
        true, // place on field when it touches the ground
        0.3 // landing dampening
        );
  }

  /**
   * Scoring targets are the 3D bounding volumes for each HUB's top opening. A piece enters LIMBO
   * when its position intersects one of these boxes.
   */
  private static List<GamePieceTarget> createFuelTargets() {
    List<GamePieceTarget> targets = new ArrayList<>();
    double half = FieldConstants.HUB_OPENING_SIZE_METERS / 2.0;

    // Blue HUB
    targets.add(
        new GamePieceTarget(
            new Translation3d(
                FieldConstants.BLUE_HUB_X_METERS - half,
                FieldConstants.BLUE_HUB_Y_METERS - half,
                FieldConstants.BLUE_HUB_Z_METERS - 0.1),
            new Translation3d(
                FieldConstants.BLUE_HUB_X_METERS + half,
                FieldConstants.BLUE_HUB_Y_METERS + half,
                FieldConstants.BLUE_HUB_Z_METERS + 0.5)));

    // Red HUB
    targets.add(
        new GamePieceTarget(
            new Translation3d(
                FieldConstants.RED_HUB_X_METERS - half,
                FieldConstants.RED_HUB_Y_METERS - half,
                FieldConstants.RED_HUB_Z_METERS - 0.1),
            new Translation3d(
                FieldConstants.RED_HUB_X_METERS + half,
                FieldConstants.RED_HUB_Y_METERS + half,
                FieldConstants.RED_HUB_Z_METERS + 0.5)));

    return targets;
  }

  // -------------------------------------------------------------------------
  // State queries (original)
  // -------------------------------------------------------------------------

  /**
   * @return true if this fuel has entered a HUB (LIMBO state, not user-controlled).
   */
  public boolean isScored() {
    return isInState(GamePieceState.LIMBO) && !isUserControlled();
  }

  /**
   * @return true if this fuel is on the field and available for intake.
   */
  public boolean isOnField() {
    return isInState(GamePieceState.ON_FIELD) && isLibraryControlled();
  }

  /**
   * @return true if this fuel is currently held by a robot.
   */
  public boolean isHeld() {
    return isInState(GamePieceState.HELD);
  }

  /**
   * @return true if this fuel is currently in flight.
   */
  public boolean isInFlight() {
    return isInState(GamePieceState.IN_FLIGHT);
  }

  // -------------------------------------------------------------------------
  // Alliance tracking
  // -------------------------------------------------------------------------

  /**
   * Records which alliance shot this piece. Call this at intake time, before the piece is launched
   * — once it reaches LIMBO its pose is (-1,-1,-1) and alliance can no longer be inferred from
   * position.
   *
   * @param isBlue true if the Blue alliance shot this piece
   */
  public void setScoringAlliance(boolean isBlue) {
    this.scoringAllianceIsBlue = isBlue;
  }

  /**
   * Returns which alliance scored this piece.
   *
   * <p>Reliable only if {@link #setScoringAlliance(boolean)} was called at intake time. If it was
   * never called, returns false (red) as a safe default — callers should always set the alliance at
   * intake rather than relying on this fallback.
   *
   * @return true if the Blue alliance scored this piece, false if Red
   */
  public boolean getScoringAlliance() {
    return scoringAllianceIsBlue;
  }

  // -------------------------------------------------------------------------
  // Newly-scored transition detection
  // -------------------------------------------------------------------------

  /**
   * Returns true exactly once per LIMBO transition — i.e., the first time this piece is observed to
   * have entered a HUB since it was last on the field.
   *
   * <p>How it works: on each call we compare the current LIMBO state against the last observed
   * state. The flag is latched true on the rising edge (not-scored → scored) and stays true until
   * {@link #clearNewlyScoredFlag()} is called. Subsequent calls before clearing return true but
   * don't re-latch.
   *
   * <p>Callers in {@code RebuiltSimArena} should call {@link #clearNewlyScoredFlag()} after
   * processing to avoid double-counting across updateAutoScores() and updateMatchPoints() within
   * the same tick.
   *
   * @return true if this piece just transitioned into a HUB this tick
   */
  public boolean isNewlyScored() {
    boolean currentlyScored = isScored();

    // Rising edge: piece just entered LIMBO
    if (currentlyScored && !wasScored) {
      newlyScoredFlag = true;
    }

    wasScored = currentlyScored;
    return newlyScoredFlag;
  }

  /**
   * Clears the newly-scored flag set by {@link #isNewlyScored()}. Call this after processing a
   * score event to prevent double-counting.
   */
  public void clearNewlyScoredFlag() {
    newlyScoredFlag = false;
  }

  public static double getDiameterMeters() {
    return FUEL_DIAMETER_M;
  }

  public static double getDiameterInches() {
    return FieldConstants.FUEL_DIAMETER_INCHES;
  }

  public static double getMassKg() {
    return FUEL_MASS_KG;
  }
}
