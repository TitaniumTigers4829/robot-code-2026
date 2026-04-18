package frc.robot.sim.sim2026;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.logging.RuntimeLog;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimGamePiece;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * The playing field for the 2026 FRC Game: REBUILT™ presented by Haas
 *
 * <p>This class represents the playing field for the 2026 FRC game, REBUILT. Features include:
 *
 * <ul>
 *   <li>HUB scoring with active/inactive states
 *   <li>TOWER climbing with three levels
 *   <li>BUMP and TRENCH obstacles
 *   <li>OUTPOST for human player interaction
 *   <li>DEPOT for FUEL storage
 * </ul>
 */
public class RebuiltSim {

  /** The main simulation arena for the 2026 REBUILT game. */
  public static class RebuiltSimArena extends SimArena {

    private boolean blueHubActive = true;
    private boolean redHubActive = true;
    private int currentShift = 0; // 0=TRANSITION, 1-4=ALLIANCE SHIFTS, 5=END GAME

    private int blueAutoScore = 0;
    private int redAutoScore = 0;
    private boolean autoComplete = false;

    private final Random random = new Random();

    private int blueMatchPoints = 0;
    private int redMatchPoints = 0;
    private int blueFuelScored = 0;
    private int redFuelScored = 0;

    private final List<Integer> blueTowerLevels = new ArrayList<>();
    private final List<Integer> redTowerLevels = new ArrayList<>();

    public RebuiltSimArena(Time period, int simulationSubTick) {
      super(new RebuiltFieldObstacleMap(), period.in(Seconds), simulationSubTick);
      RuntimeLog.info("REBUILT 2026 Simulation Arena initialized");
    }

    @Override
    protected void placeGamePiecesOnField() {
      gamePieces.forEach(gp -> gp.withLib(SimGamePiece::delete));
      gamePieces.clear();

      placeFuelInDepot(true);
      placeFuelInDepot(false);
      placeFuelInOutpostChute(true);
      placeFuelInOutpostChute(false);
      placeFuelInNeutralZone();

      RuntimeLog.info("REBUILT: Game pieces placed on field");
    }

    private void placeFuelInDepot(boolean isBlue) {
      Translation2d depotPos =
          isBlue
              ? new Translation2d(
                  FieldConstants.BLUE_DEPOT_X_METERS, FieldConstants.BLUE_DEPOT_Y_METERS)
              : new Translation2d(
                  FieldConstants.RED_DEPOT_X_METERS, FieldConstants.RED_DEPOT_Y_METERS);

      for (int i = 0; i < FieldConstants.FUEL_PER_DEPOT; i++) {
        double xOffset = (random.nextDouble() - 0.5) * 0.3;
        double yOffset = (random.nextDouble() - 0.5) * 0.3;
        Fuel fuel = new Fuel(this, depotPos.plus(new Translation2d(xOffset, yOffset)));
        fuel.releaseControl();
        gamePieces.add(fuel);
      }
    }

    private void placeFuelInOutpostChute(boolean isBlue) {
      Translation2d outpostPos =
          isBlue
              ? new Translation2d(
                  FieldConstants.BLUE_OUTPOST_X_METERS, FieldConstants.BLUE_OUTPOST_CENTER_Y_METERS)
              : new Translation2d(
                  FieldConstants.RED_OUTPOST_X_METERS, FieldConstants.RED_OUTPOST_CENTER_Y_METERS);

      for (int i = 0; i < FieldConstants.FUEL_PER_OUTPOST_CHUTE; i++) {
        double xOffset = (random.nextDouble() - 0.5) * 0.4;
        double yOffset = (random.nextDouble() - 0.5) * 0.4;
        Fuel fuel = new Fuel(this, outpostPos.plus(new Translation2d(xOffset, yOffset)));
        fuel.releaseControl();
        gamePieces.add(fuel);
      }
    }

    private void placeFuelInNeutralZone() {
      double startX = FieldConstants.ALLIANCE_ZONE_DEPTH_METERS;
      double endX = FieldConstants.FIELD_LENGTH_METERS - FieldConstants.ALLIANCE_ZONE_DEPTH_METERS;
      double width = FieldConstants.FIELD_WIDTH_METERS;

      int fuelCount = 380 + random.nextInt(49);
      for (int i = 0; i < fuelCount; i++) {
        double x = startX + random.nextDouble() * (endX - startX);
        double y = random.nextDouble() * width;
        Fuel fuel = new Fuel(this, new Translation2d(x, y));
        fuel.releaseControl();
        gamePieces.add(fuel);
      }
    }

    @Override
    protected void competitionPeriodic() {
      if (!DriverStation.isEnabled()) return;

      double matchTime = Timer.getMatchTime();

      if (DriverStation.isAutonomous() && !autoComplete) {
        updateAutoScores();
      } else if (DriverStation.isAutonomousEnabled() && !autoComplete && matchTime <= 0.0) {
        finalizeAutoResults();
      }

      if (DriverStation.isTeleopEnabled()) {
        updateHubStatus(matchTime);
      }

      updateMatchPoints();
      checkTowerClimbs();
    }

    /**
     * Updates AUTO scores by counting fuel that enters a scoring target (LIMBO state).
     *
     * <p>IMPORTANT: We track which alliance scored at INTAKE time via {@link
     * Fuel#getScoringAlliance()}, NOT from fuel.pose() at score time. Scored fuel transitions to
     * LIMBO with pose (-1,-1,-1), making pose-based alliance detection impossible once scored.
     *
     * <p>To do this correctly, call {@link Fuel#setScoringAlliance(boolean)} when a robot intakes a
     * piece, before it is shot. For now we approximate using the fuel's last known field position.
     */
    private void updateAutoScores() {
      for (SimGamePiece gp : gamePieces) {
        if (gp instanceof Fuel fuel && fuel.isNewlyScored()) {
          // Use the scoring alliance recorded at intake time, not the LIMBO pose.
          // See Fuel.getScoringAlliance() — falls back to last known position if not set.
          boolean isBlue = fuel.getScoringAlliance();
          if (isBlue) {
            blueAutoScore++;
            blueFuelScored++;
          } else {
            redAutoScore++;
            redFuelScored++;
          }
          fuel.clearNewlyScoredFlag();
        }
      }
    }

    private void finalizeAutoResults() {
      autoComplete = true;
      RuntimeLog.info("REBUILT AUTO Complete — Blue: " + blueAutoScore + "  Red: " + redAutoScore);

      if (blueAutoScore > redAutoScore) {
        blueHubActive = false;
        redHubActive = true;
        RuntimeLog.info("REBUILT: Blue HUB inactive in SHIFT 1 (Blue scored more in AUTO)");
      } else if (redAutoScore > blueAutoScore) {
        blueHubActive = true;
        redHubActive = false;
        RuntimeLog.info("REBUILT: Red HUB inactive in SHIFT 1 (Red scored more in AUTO)");
      } else {
        boolean randomBlueInactive = random.nextBoolean();
        blueHubActive = !randomBlueInactive;
        redHubActive = randomBlueInactive;
        RuntimeLog.info(
            "REBUILT: AUTO tie — random selection: "
                + (randomBlueInactive ? "Blue" : "Red")
                + " HUB inactive in SHIFT 1");
      }
      currentShift = 0;
    }

    private void updateHubStatus(double matchTime) {
      double elapsed = FieldConstants.MATCH_DURATION_SECONDS - matchTime;

      if (elapsed < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS) {
        if (currentShift != 0) {
          currentShift = 0;
          blueHubActive = true;
          redHubActive = true;
          RuntimeLog.debug("REBUILT: TRANSITION SHIFT — both HUBs active");
        }
      } else if (elapsed
          < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS
              + FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
        if (currentShift != 1) {
          currentShift = 1;
          RuntimeLog.debug("REBUILT: SHIFT 1 — Blue=" + blueHubActive + " Red=" + redHubActive);
        }
      } else if (elapsed
          < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS
              + 2 * FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
        if (currentShift != 2) {
          currentShift = 2;
          blueHubActive = !blueHubActive;
          redHubActive = !redHubActive;
          RuntimeLog.debug("REBUILT: SHIFT 2 — Blue=" + blueHubActive + " Red=" + redHubActive);
        }
      } else if (elapsed
          < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS
              + 3 * FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
        if (currentShift != 3) {
          currentShift = 3;
          blueHubActive = !blueHubActive;
          redHubActive = !redHubActive;
          RuntimeLog.debug("REBUILT: SHIFT 3 — Blue=" + blueHubActive + " Red=" + redHubActive);
        }
      } else if (elapsed
          < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS
              + 4 * FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
        if (currentShift != 4) {
          currentShift = 4;
          blueHubActive = !blueHubActive;
          redHubActive = !redHubActive;
          RuntimeLog.debug("REBUILT: SHIFT 4 — Blue=" + blueHubActive + " Red=" + redHubActive);
        }
      } else {
        if (currentShift != 5) {
          currentShift = 5;
          blueHubActive = true;
          redHubActive = true;
          RuntimeLog.debug("REBUILT: END GAME — both HUBs active");
        }
      }
    }

    private void updateMatchPoints() {
      for (SimGamePiece gp : gamePieces) {
        if (gp instanceof Fuel fuel && fuel.isNewlyScored()) {
          boolean isBlue = fuel.getScoringAlliance();
          if ((isBlue && blueHubActive) || (!isBlue && redHubActive)) {
            if (isBlue) blueMatchPoints += FieldConstants.FUEL_SCORE_ACTIVE_HUB;
            else redMatchPoints += FieldConstants.FUEL_SCORE_ACTIVE_HUB;
          }
        }
      }
    }

    private void checkTowerClimbs() {
      // Only relevant in END GAME (last 30 seconds)
      if (Timer.getMatchTime() > FieldConstants.END_GAME_START_TIME) return;
      // TODO: implement robot-position-based climb detection against tower rung heights.
    }

    public void recordAutoScore(boolean isBlue, int count) {
      if (isBlue) blueAutoScore += count;
      else redAutoScore += count;
    }

    public boolean isBlueHubActive() {
      return blueHubActive;
    }

    public boolean isRedHubActive() {
      return redHubActive;
    }

    public int getCurrentShift() {
      return currentShift;
    }

    public int getBlueMatchPoints() {
      return blueMatchPoints;
    }

    public int getRedMatchPoints() {
      return redMatchPoints;
    }

    public int getBlueFuelScored() {
      return blueFuelScored;
    }

    public int getRedFuelScored() {
      return redFuelScored;
    }

    public void resetScores() {
      blueAutoScore = 0;
      redAutoScore = 0;
      blueMatchPoints = 0;
      redMatchPoints = 0;
      blueFuelScored = 0;
      redFuelScored = 0;
      blueTowerLevels.clear();
      redTowerLevels.clear();
      autoComplete = false;
      blueHubActive = true;
      redHubActive = true;
      currentShift = 0;
      RuntimeLog.info("REBUILT: Scores reset for new match");
    }

    public Fuel createFuelForHumanPlayer(boolean isBlue) {
      Translation2d outpostPos =
          isBlue
              ? new Translation2d(
                  FieldConstants.BLUE_OUTPOST_X_METERS, FieldConstants.BLUE_OUTPOST_CENTER_Y_METERS)
              : new Translation2d(
                  FieldConstants.RED_OUTPOST_X_METERS, FieldConstants.RED_OUTPOST_CENTER_Y_METERS);
      Fuel fuel = new Fuel(this, outpostPos);
      fuel.userControlled();
      gamePieces.add(fuel);
      return fuel;
    }
  }

  /** Field obstacle map for the 2026 REBUILT game. */
  public static class RebuiltFieldObstacleMap extends SimArena.FieldMap {

    public RebuiltFieldObstacleMap() {
      super();
      addFieldBorders();
      addBumpObstacles();
      addTrenchObstacles();
      addHubObstacles();
      addDepotObstacles();
      addTowerObstacles();
      addOutpostObstacles();
      RuntimeLog.info("REBUILT 2026 Field Obstacle Map created");
    }

    private void addFieldBorders() {
      addBorderLine(
          new Translation2d(0, 0), new Translation2d(0, FieldConstants.FIELD_WIDTH_METERS));
      addBorderLine(
          new Translation2d(FieldConstants.FIELD_LENGTH_METERS, 0),
          new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS));
      addBorderLine(
          new Translation2d(0, FieldConstants.FIELD_WIDTH_METERS),
          new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS));
      addBorderLine(
          new Translation2d(0, 0), new Translation2d(FieldConstants.FIELD_LENGTH_METERS, 0));
    }

    /**
     * Adds BUMP obstacles.
     *
     * <p>Each alliance has two BUMPs, one on the audience side and one on the scoring-table side of
     * their HUB. They are centered on the hub X and offset in Y from the hub faces.
     *
     * <p>Source: GE-26100 for dimensions; FE-2026 Sheet 3 ref dims for Y placement.
     */
    private void addBumpObstacles() {
      // Blue — audience side
      addRectangularObstacle(
          FieldConstants.BUMP_WIDTH_METERS,
          FieldConstants.BUMP_DEPTH_METERS,
          new Pose2d(
              FieldConstants.BLUE_BUMP_AUDIENCE_CENTER_X_METERS,
              FieldConstants.BLUE_BUMP_AUDIENCE_CENTER_Y_METERS,
              new Rotation2d()));

      // Blue — scoring-table side
      addRectangularObstacle(
          FieldConstants.BUMP_WIDTH_METERS,
          FieldConstants.BUMP_DEPTH_METERS,
          new Pose2d(
              FieldConstants.BLUE_BUMP_SCORINGTABLE_CENTER_X_METERS,
              FieldConstants.BLUE_BUMP_SCORINGTABLE_CENTER_Y_METERS,
              new Rotation2d()));

      // Red — audience side
      addRectangularObstacle(
          FieldConstants.BUMP_WIDTH_METERS,
          FieldConstants.BUMP_DEPTH_METERS,
          new Pose2d(
              FieldConstants.RED_BUMP_AUDIENCE_CENTER_X_METERS,
              FieldConstants.RED_BUMP_AUDIENCE_CENTER_Y_METERS,
              new Rotation2d()));

      // Red — scoring-table side
      addRectangularObstacle(
          FieldConstants.BUMP_WIDTH_METERS,
          FieldConstants.BUMP_DEPTH_METERS,
          new Pose2d(
              FieldConstants.RED_BUMP_SCORINGTABLE_CENTER_X_METERS,
              FieldConstants.RED_BUMP_SCORINGTABLE_CENTER_Y_METERS,
              new Rotation2d()));
    }

    /**
     * Adds TRENCH obstacles.
     *
     * <p>Each TRENCH runs in the X direction from the alliance wall to the hub face, at a fixed Y
     * near the audience or scoring-table wall. The modeled width in Y is small (the beam/rail
     * cross-section). The clearance height is 22.25" — relevant for future 3D sim, not 2D physics.
     *
     * <p>Source: FE-2026 Sheet 11 (AprilTag Y positions) for rail Y locations; Sheet 3 for
     * approximate X extent.
     */
    private void addTrenchObstacles() {
      // Trench rail Y-extent in the obstacle map — model as a 4" wide beam.
      // This is a narrow obstacle; the real exclusion is the robot height vs
      // TRENCH_CLEARANCE_HEIGHT_INCHES = 22.25". Flag for future 3D upgrade.
      final double TRENCH_RAIL_THICKNESS_METERS = 0.10; // ~4" placeholder

      // Blue — audience side rail
      addRectangularObstacle(
          FieldConstants.BLUE_TRENCH_LENGTH_X_METERS,
          TRENCH_RAIL_THICKNESS_METERS,
          new Pose2d(
              FieldConstants.BLUE_TRENCH_CENTER_X_METERS,
              FieldConstants.TRENCH_AUDIENCE_SIDE_Y_METERS,
              new Rotation2d()));

      // Blue — scoring-table side rail
      addRectangularObstacle(
          FieldConstants.BLUE_TRENCH_LENGTH_X_METERS,
          TRENCH_RAIL_THICKNESS_METERS,
          new Pose2d(
              FieldConstants.BLUE_TRENCH_CENTER_X_METERS,
              FieldConstants.TRENCH_SCORINGTABLE_SIDE_Y_METERS,
              new Rotation2d()));

      // Red — audience side rail
      addRectangularObstacle(
          FieldConstants.RED_TRENCH_LENGTH_X_METERS,
          TRENCH_RAIL_THICKNESS_METERS,
          new Pose2d(
              FieldConstants.RED_TRENCH_CENTER_X_METERS,
              FieldConstants.TRENCH_AUDIENCE_SIDE_Y_METERS,
              new Rotation2d()));

      // Red — scoring-table side rail
      addRectangularObstacle(
          FieldConstants.RED_TRENCH_LENGTH_X_METERS,
          TRENCH_RAIL_THICKNESS_METERS,
          new Pose2d(
              FieldConstants.RED_TRENCH_CENTER_X_METERS,
              FieldConstants.TRENCH_SCORINGTABLE_SIDE_Y_METERS,
              new Rotation2d()));
    }

    private void addHubObstacles() {
      addRectangularObstacle(
          FieldConstants.HUB_WIDTH_METERS,
          FieldConstants.HUB_DEPTH_METERS,
          new Pose2d(
              FieldConstants.BLUE_HUB_X_METERS,
              FieldConstants.BLUE_HUB_Y_METERS,
              new Rotation2d()));

      addRectangularObstacle(
          FieldConstants.HUB_WIDTH_METERS,
          FieldConstants.HUB_DEPTH_METERS,
          new Pose2d(
              FieldConstants.RED_HUB_X_METERS, FieldConstants.RED_HUB_Y_METERS, new Rotation2d()));
    }

    private void addDepotObstacles() {
      // Depot is an L-shaped weldment. Model as two rectangular arms.
      // Long arm (along Y)
      addRectangularObstacle(
          Units.inchesToMeters(3.0), // thin in X
          FieldConstants.DEPOT_ARM_LONG_METERS, // 42" along Y
          new Pose2d(
              FieldConstants.BLUE_DEPOT_X_METERS,
              FieldConstants.BLUE_DEPOT_Y_METERS + FieldConstants.DEPOT_ARM_LONG_METERS / 2,
              new Rotation2d()));

      // Short arm (along X)
      addRectangularObstacle(
          FieldConstants.DEPOT_ARM_SHORT_METERS, // 27" along X
          Units.inchesToMeters(3.0), // thin in Y
          new Pose2d(
              FieldConstants.BLUE_DEPOT_X_METERS + FieldConstants.DEPOT_ARM_SHORT_METERS / 2,
              FieldConstants.BLUE_DEPOT_Y_METERS,
              new Rotation2d()));

      // Red DEPOT (mirror)
      addRectangularObstacle(
          Units.inchesToMeters(3.0),
          FieldConstants.DEPOT_ARM_LONG_METERS,
          new Pose2d(
              FieldConstants.RED_DEPOT_X_METERS,
              FieldConstants.RED_DEPOT_Y_METERS + FieldConstants.DEPOT_ARM_LONG_METERS / 2,
              new Rotation2d()));

      addRectangularObstacle(
          FieldConstants.DEPOT_ARM_SHORT_METERS,
          Units.inchesToMeters(3.0),
          new Pose2d(
              FieldConstants.RED_DEPOT_X_METERS - FieldConstants.DEPOT_ARM_SHORT_METERS / 2,
              FieldConstants.RED_DEPOT_Y_METERS,
              new Rotation2d()));
    }

    /**
     * Adds TOWER obstacles.
     *
     * <p>The TOWER sits against the alliance wall. Modeled as a rectangle. Dimensions from GE-26500
     * sheet 2: 40" wide (Y) × 32.25" deep (X).
     */
    private void addTowerObstacles() {
      // Blue TOWER — at Blue wall, centered at BLUE_TOWER_CENTER_Y
      addRectangularObstacle(
          FieldConstants.TOWER_DEPTH_METERS, // 32.25" in X (into field)
          FieldConstants.TOWER_WIDTH_METERS, // 40.00" in Y (across rungs)
          new Pose2d(
              FieldConstants.BLUE_TOWER_X_METERS + FieldConstants.TOWER_DEPTH_METERS / 2,
              FieldConstants.BLUE_TOWER_CENTER_Y_METERS,
              new Rotation2d()));

      // Red TOWER — at Red wall
      addRectangularObstacle(
          FieldConstants.TOWER_DEPTH_METERS,
          FieldConstants.TOWER_WIDTH_METERS,
          new Pose2d(
              FieldConstants.RED_TOWER_X_METERS - FieldConstants.TOWER_DEPTH_METERS / 2,
              FieldConstants.RED_TOWER_CENTER_Y_METERS,
              new Rotation2d()));
    }

    /**
     * Adds OUTPOST obstacles.
     *
     * <p>Modeled as a rectangle for the outpost frame footprint. Blue OUTPOST is near the
     * scoring-table wall (high Y); Red OUTPOST is near the audience wall (low Y).
     */
    private void addOutpostObstacles() {
      // Blue OUTPOST — scoring-table side (high Y)
      addRectangularObstacle(
          FieldConstants.OUTPOST_DEPTH_METERS, // 17" in X (into field)
          FieldConstants.OUTPOST_WIDTH_METERS, // 32" in Y (face width)
          new Pose2d(
              FieldConstants.BLUE_OUTPOST_X_METERS + FieldConstants.OUTPOST_DEPTH_METERS / 2,
              FieldConstants.BLUE_OUTPOST_CENTER_Y_METERS,
              new Rotation2d()));

      // Red OUTPOST — audience side (low Y)
      addRectangularObstacle(
          FieldConstants.OUTPOST_DEPTH_METERS,
          FieldConstants.OUTPOST_WIDTH_METERS,
          new Pose2d(
              FieldConstants.RED_OUTPOST_X_METERS - FieldConstants.OUTPOST_DEPTH_METERS / 2,
              FieldConstants.RED_OUTPOST_CENTER_Y_METERS,
              new Rotation2d()));
    }
  }
}
