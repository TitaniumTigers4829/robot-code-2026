package frc.robot.sim.sim2026;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.logging.RuntimeLog;
import frc.robot.extras.math.forces.Velocity2d;
import frc.robot.extras.math.mathutils.GeomUtil;
import frc.robot.extras.util.FrcBody;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimGamePiece;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

/**
 * The playing field for the 2026 FRC Game: REBUILT™ presented by Haas
 * 
 * <p>This class represents the playing field for the 2026 FRC game, REBUILT.
 * Features include:
 * <ul>
 *   <li>HUB scoring with active/inactive states</li>
 *   <li>TOWER climbing with three levels</li>
 *   <li>BUMP and TRENCH obstacles</li>
 *   <li>OUTPOST for human player interaction</li>
 *   <li>DEPOT for FUEL storage</li>
 * </ul>
 */
public class RebuiltSim {
    
    /**
     * The main simulation arena for the 2026 REBUILT game.
     */
    public static class RebuiltSimArena extends SimArena {
        
        // Track HUB active states based on AUTO results
        private boolean blueHubActive = true;
        private boolean redHubActive = true;
        private int currentShift = 0; // 0 = TRANSITION, 1-4 = ALLIANCE SHIFTS, 5 = END GAME
        private double previousScoreTimeSeconds = 0;
        
        // AUTO result tracking
        private int blueAutoScore = 0;
        private int redAutoScore = 0;
        private boolean autoComplete = false;
        
        // Random for tie-breaking
        private final Random random = new Random();
        
        // Scoring tracking
        private int blueMatchPoints = 0;
        private int redMatchPoints = 0;
        private int blueFuelScored = 0;
        private int redFuelScored = 0;
        
        // Tower climb tracking
        private final List<Integer> blueTowerLevels = new ArrayList<>();
        private final List<Integer> redTowerLevels = new ArrayList<>();
        
        /**
         * Creates a new REBUILT simulation arena.
         *
         * @param period The simulation period
         * @param simulationSubTick The number of sub-ticks per period
         */
        public RebuiltSimArena(Time period, int simulationSubTick) {
            super(new RebuiltFieldObstacleMap(), period.in(Seconds), simulationSubTick);
            RuntimeLog.info("REBUILT 2026 Simulation Arena initialized");
        }
        
        @Override
        protected void placeGamePiecesOnField() {
            // Clear any existing game pieces
            gamePieces.forEach(gp -> gp.withLib(SimGamePiece::delete));
            gamePieces.clear();
            
            // Place FUEL in DEPOTs (24 each)
            placeFuelInDepot(true);  // Blue depot
            placeFuelInDepot(false); // Red depot
            
            // Place FUEL in OUTPOST CHUTEs (24 each)
            placeFuelInOutpostChute(true);  // Blue outpost
            placeFuelInOutpostChute(false); // Red outpost
            
            // Place remaining FUEL in NEUTRAL ZONE (360-408 pieces)
            placeFuelInNeutralZone();
            
            RuntimeLog.info("REBUILT: Game pieces placed on field");
        }
        
        /**
         * Places FUEL in a DEPOT.
         */
        private void placeFuelInDepot(boolean isBlue) {
            Translation2d depotPos = isBlue ? 
                new Translation2d(FieldConstants.BLUE_DEPOT_X_METERS, FieldConstants.BLUE_DEPOT_Y_METERS) :
                new Translation2d(FieldConstants.RED_DEPOT_X_METERS, FieldConstants.RED_DEPOT_Y_METERS);
            
            for (int i = 0; i < FieldConstants.FUEL_PER_DEPOT; i++) {
                // Add slight random offset within depot
                double xOffset = (random.nextDouble() - 0.5) * 0.3;
                double yOffset = (random.nextDouble() - 0.5) * 0.3;
                Translation2d fuelPos = depotPos.plus(new Translation2d(xOffset, yOffset));
                
                Fuel fuel = new Fuel(this, fuelPos);
                fuel.releaseControl();
                gamePieces.add(fuel);
            }
        }
        
        /**
         * Places FUEL in an OUTPOST CHUTE.
         */
        private void placeFuelInOutpostChute(boolean isBlue) {
            Translation2d outpostPos = isBlue ? 
                new Translation2d(FieldConstants.BLUE_OUTPOST_X_METERS, FieldConstants.BLUE_OUTPOST_Y_METERS) :
                new Translation2d(FieldConstants.RED_OUTPOST_X_METERS, FieldConstants.RED_OUTPOST_Y_METERS);
            
            // CHUTE opening is at 28.1" height, but we place on ground for simplicity
            // The human player will "feed" them through the chute during gameplay
            for (int i = 0; i < FieldConstants.FUEL_PER_OUTPOST_CHUTE; i++) {
                double xOffset = (random.nextDouble() - 0.5) * 0.4;
                double yOffset = (random.nextDouble() - 0.5) * 0.4;
                Translation2d fuelPos = outpostPos.plus(new Translation2d(xOffset, yOffset));
                
                Fuel fuel = new Fuel(this, fuelPos);
                fuel.releaseControl();
                gamePieces.add(fuel);
            }
        }
        
        /**
         * Places FUEL in the NEUTRAL ZONE.
         */
        private void placeFuelInNeutralZone() {
            // NEUTRAL ZONE is 283" deep x 317.7" wide
            double neutralZoneStartX = FieldConstants.ALLIANCE_ZONE_DEPTH_METERS;
            double neutralZoneEndX = FieldConstants.FIELD_LENGTH_METERS - FieldConstants.ALLIANCE_ZONE_DEPTH_METERS;
            double neutralZoneWidth = FieldConstants.FIELD_WIDTH_METERS;
            
            // Approx 380 fuel in neutral zone
            int fuelCount = 380 + random.nextInt(49); // 360-408 range
            
            for (int i = 0; i < fuelCount; i++) {
                double x = neutralZoneStartX + random.nextDouble() * (neutralZoneEndX - neutralZoneStartX);
                double y = random.nextDouble() * neutralZoneWidth;
                
                Fuel fuel = new Fuel(this, new Translation2d(x, y));
                fuel.releaseControl();
                gamePieces.add(fuel);
            }
        }
        
        @Override
        protected void competitionPeriodic() {
            if (!DriverStation.isEnabled()) return;
            
            double matchTime = Timer.getMatchTime();
            
            // Track AUTO scores to determine HUB status for ALLIANCE SHIFTS
            if (DriverStation.isAutonomous() && !autoComplete) {
                updateAutoScores();
            } else if (DriverStation.isAutonomousEnabled() && !autoComplete && matchTime <= 0.0) {
                // AUTO just ended
                finalizeAutoResults();
            }
            
            // Track TELEOP shift timing
            if (DriverStation.isTeleopEnabled()) {
                updateHubStatus(matchTime);
            }
            
            // Update match points based on scored fuel
            updateMatchPoints();
            
            // Check for tower climbs
            checkTowerClimbs();
        }
        
        /**
         * Updates AUTO scores by counting fuel that enters scoring targets.
         */
        private void updateAutoScores() {
            for (SimGamePiece gp : gamePieces) {
                if (gp instanceof Fuel fuel && fuel.isScored()) {
                    // Determine which alliance scored based on position
                    Pose3d pose = fuel.pose();
                    if (pose.getX() < FieldConstants.CENTER_LINE_X_METERS) {
                        blueAutoScore++;
                        blueFuelScored++;
                    } else {
                        redAutoScore++;
                        redFuelScored++;
                    }
                }
            }
        }
        
        /**
         * Finalizes AUTO results and sets initial HUB status.
         */
        private void finalizeAutoResults() {
            autoComplete = true;
            
            RuntimeLog.info("REBUILT AUTO Complete - Blue: " + blueAutoScore + " Red: " + redAutoScore);
            
            // Determine HUB status for TELEOP based on AUTO results
            // The alliance with MORE FUEL in AUTO has their HUB INACTIVE in SHIFT 1
            if (blueAutoScore > redAutoScore) {
                // Blue scored more - their hub inactive in SHIFT 1
                blueHubActive = false;
                redHubActive = true;
                RuntimeLog.info("REBUILT: Blue hub inactive in SHIFT 1 (Blue scored more)");
            } else if (redAutoScore > blueAutoScore) {
                // Red scored more - their hub inactive in SHIFT 1
                blueHubActive = true;
                redHubActive = false;
                RuntimeLog.info("REBUILT: Red hub inactive in SHIFT 1 (Red scored more)");
            } else {
                // Tie - randomly select (simulate FMS random selection)
                boolean randomBlueInactive = random.nextBoolean();
                blueHubActive = !randomBlueInactive;
                redHubActive = randomBlueInactive;
                RuntimeLog.info("REBUILT: Tie - Random selection: " + 
                    (randomBlueInactive ? "Blue" : "Red") + " hub inactive in SHIFT 1");
            }
            
            currentShift = 0; // TRANSITION SHIFT
        }
        
        /**
         * Updates HUB active status based on current match time.
         */
        private void updateHubStatus(double matchTime) {
            // Convert match time (seconds remaining) to match time elapsed
            double timeElapsed = FieldConstants.MATCH_DURATION_SECONDS - matchTime;
            
            // TRANSITION SHIFT: 0-10 seconds (both hubs active)
            if (timeElapsed < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS) {
                if (currentShift != 0) {
                    currentShift = 0;
                    blueHubActive = true;
                    redHubActive = true;
                    RuntimeLog.debug("REBUILT: TRANSITION SHIFT - Both hubs active");
                }
            }
            // SHIFT 1: 10-35 seconds
            else if (timeElapsed < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS + FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
                if (currentShift != 1) {
                    currentShift = 1;
                    // Status already set from AUTO results
                    RuntimeLog.debug("REBUILT: SHIFT 1 - Blue active: " + blueHubActive + 
                                   ", Red active: " + redHubActive);
                }
            }
            // SHIFT 2: 35-60 seconds
            else if (timeElapsed < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS + 2 * FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
                if (currentShift != 2) {
                    currentShift = 2;
                    // Alternate status
                    blueHubActive = !blueHubActive;
                    redHubActive = !redHubActive;
                    RuntimeLog.debug("REBUILT: SHIFT 2 - Blue active: " + blueHubActive + 
                                   ", Red active: " + redHubActive);
                }
            }
            // SHIFT 3: 60-85 seconds
            else if (timeElapsed < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS + 3 * FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
                if (currentShift != 3) {
                    currentShift = 3;
                    // Alternate status
                    blueHubActive = !blueHubActive;
                    redHubActive = !redHubActive;
                    RuntimeLog.debug("REBUILT: SHIFT 3 - Blue active: " + blueHubActive + 
                                   ", Red active: " + redHubActive);
                }
            }
            // SHIFT 4: 85-110 seconds
            else if (timeElapsed < FieldConstants.TRANSITION_SHIFT_DURATION_SECONDS + 4 * FieldConstants.ALLIANCE_SHIFT_DURATION_SECONDS) {
                if (currentShift != 4) {
                    currentShift = 4;
                    // Alternate status
                    blueHubActive = !blueHubActive;
                    redHubActive = !redHubActive;
                    RuntimeLog.debug("REBUILT: SHIFT 4 - Blue active: " + blueHubActive + 
                                   ", Red active: " + redHubActive);
                }
            }
            // END GAME: 110-140 seconds (both hubs active)
            else {
                if (currentShift != 5) {
                    currentShift = 5;
                    blueHubActive = true;
                    redHubActive = true;
                    RuntimeLog.debug("REBUILT: END GAME - Both hubs active");
                }
            }
        }
        
        /**
         * Updates match points based on scored fuel.
         */
        private void updateMatchPoints() {
            for (SimGamePiece gp : gamePieces) {
                if (gp instanceof Fuel fuel && fuel.isScored()) {
                    Pose3d pose = fuel.pose();
                    boolean isBlueSide = pose.getX() < FieldConstants.CENTER_LINE_X_METERS;
                    
                    // Only count if the hub is active when scored
                    if ((isBlueSide && blueHubActive) || (!isBlueSide && redHubActive)) {
                        if (isBlueSide) {
                            blueMatchPoints += FieldConstants.FUEL_SCORE_ACTIVE_HUB;
                        } else {
                            redMatchPoints += FieldConstants.FUEL_SCORE_ACTIVE_HUB;
                        }
                    }
                }
            }
        }
        
        /**
         * Checks for robots climbing the TOWER.
         * In a real simulation, this would interface with robot simulation.
         */
        private void checkTowerClimbs() {
            double matchTime = Timer.getMatchTime();
            
            // Only check in END GAME (last 30 seconds)
            if (matchTime > FieldConstants.END_GAME_START_TIME) return;
            
            // This would normally check robot positions relative to tower rungs
            // For now, we'll simulate with a simple approach
        }
        
        /**
         * Records a FUEL score during AUTO (called by robot simulation).
         */
        public void recordAutoScore(boolean isBlue, int count) {
            if (isBlue) {
                blueAutoScore += count;
            } else {
                redAutoScore += count;
            }
        }
        
        /**
         * @return true if the blue HUB is currently active
         */
        public boolean isBlueHubActive() {
            return blueHubActive;
        }
        
        /**
         * @return true if the red HUB is currently active
         */
        public boolean isRedHubActive() {
            return redHubActive;
        }
        
        /**
         * @return The current SHIFT (0=TRANSITION, 1-4=ALLIANCE SHIFTS, 5=END GAME)
         */
        public int getCurrentShift() {
            return currentShift;
        }
        
        /**
         * @return Blue alliance match points
         */
        public int getBlueMatchPoints() {
            return blueMatchPoints;
        }
        
        /**
         * @return Red alliance match points
         */
        public int getRedMatchPoints() {
            return redMatchPoints;
        }
        
        /**
         * @return Blue alliance fuel scored
         */
        public int getBlueFuelScored() {
            return blueFuelScored;
        }
        
        /**
         * @return Red alliance fuel scored
         */
        public int getRedFuelScored() {
            return redFuelScored;
        }
        
        /**
         * Resets scores for a new match.
         */
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
        
        /**
         * Creates a new FUEL piece for human player input.
         */
        public Fuel createFuelForHumanPlayer(boolean isBlue) {
            Translation2d outpostPos = isBlue ? 
                new Translation2d(FieldConstants.BLUE_OUTPOST_X_METERS, FieldConstants.BLUE_OUTPOST_Y_METERS) :
                new Translation2d(FieldConstants.RED_OUTPOST_X_METERS, FieldConstants.RED_OUTPOST_Y_METERS);
            
            Fuel fuel = new Fuel(this, outpostPos);
            fuel.userControlled(); // Give control to human player
            gamePieces.add(fuel);
            
            return fuel;
        }
    }
    
    /**
     * Field obstacle map for the 2026 REBUILT game.
     */
    public static class RebuiltFieldObstacleMap extends SimArena.FieldMap {
        
        public RebuiltFieldObstacleMap() {
            super();
            
            // Add field borders
            addFieldBorders();
            
            // Add BUMP obstacles
            addBumpObstacles();
            
            // Add TRENCH obstacles
            addTrenchObstacles();
            
            // Add HUB obstacles
            addHubObstacles();
            
            // Add DEPOT obstacles
            addDepotObstacles();
            
            // Add TOWER obstacles
            addTowerObstacles();
            
            // Add OUTPOST obstacles
            addOutpostObstacles();
            
            RuntimeLog.info("REBUILT 2026 Field Obstacle Map created");
        }
        
        /**
         * Adds the field border walls.
         */
        private void addFieldBorders() {
            // Blue alliance wall (x = 0)
            addBorderLine(
                new Translation2d(0, 0),
                new Translation2d(0, FieldConstants.FIELD_WIDTH_METERS)
            );
            
            // Red alliance wall (x = field length)
            addBorderLine(
                new Translation2d(FieldConstants.FIELD_LENGTH_METERS, 0),
                new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS)
            );
            
            // Upper wall (scoring table side, y = field width)
            addBorderLine(
                new Translation2d(0, FieldConstants.FIELD_WIDTH_METERS),
                new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS)
            );
            
            // Lower wall (audience side, y = 0)
            addBorderLine(
                new Translation2d(0, 0),
                new Translation2d(FieldConstants.FIELD_LENGTH_METERS, 0)
            );
        }
        
        /**
         * Adds BUMP obstacles.
         */
        private void addBumpObstacles() {
            // Blue BUMP left
            addRectangularObstacle(
                FieldConstants.BUMP_WIDTH_METERS,
                FieldConstants.BUMP_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_BUMP_LEFT_X_METERS + FieldConstants.BUMP_WIDTH_METERS/2,
                    FieldConstants.BLUE_BUMP_LEFT_Y_METERS + FieldConstants.BUMP_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Blue BUMP right
            addRectangularObstacle(
                FieldConstants.BUMP_WIDTH_METERS,
                FieldConstants.BUMP_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_BUMP_RIGHT_X_METERS + FieldConstants.BUMP_WIDTH_METERS/2,
                    FieldConstants.BLUE_BUMP_RIGHT_Y_METERS + FieldConstants.BUMP_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Red BUMP left
            addRectangularObstacle(
                FieldConstants.BUMP_WIDTH_METERS,
                FieldConstants.BUMP_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_BUMP_LEFT_X_METERS + FieldConstants.BUMP_WIDTH_METERS/2,
                    FieldConstants.RED_BUMP_LEFT_Y_METERS + FieldConstants.BUMP_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Red BUMP right
            addRectangularObstacle(
                FieldConstants.BUMP_WIDTH_METERS,
                FieldConstants.BUMP_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_BUMP_RIGHT_X_METERS + FieldConstants.BUMP_WIDTH_METERS/2,
                    FieldConstants.RED_BUMP_RIGHT_Y_METERS + FieldConstants.BUMP_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
        }
        
        /**
         * Adds TRENCH obstacles.
         */
        private void addTrenchObstacles() {
            // Blue TRENCH left
            addRectangularObstacle(
                FieldConstants.TRENCH_WIDTH_METERS,
                FieldConstants.TRENCH_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_TRENCH_LEFT_X_METERS + FieldConstants.TRENCH_WIDTH_METERS/2,
                    FieldConstants.BLUE_TRENCH_LEFT_Y_METERS + FieldConstants.TRENCH_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Blue TRENCH right
            addRectangularObstacle(
                FieldConstants.TRENCH_WIDTH_METERS,
                FieldConstants.TRENCH_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_TRENCH_RIGHT_X_METERS + FieldConstants.TRENCH_WIDTH_METERS/2,
                    FieldConstants.BLUE_TRENCH_RIGHT_Y_METERS + FieldConstants.TRENCH_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Red TRENCH left
            addRectangularObstacle(
                FieldConstants.TRENCH_WIDTH_METERS,
                FieldConstants.TRENCH_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_TRENCH_LEFT_X_METERS + FieldConstants.TRENCH_WIDTH_METERS/2,
                    FieldConstants.RED_TRENCH_LEFT_Y_METERS + FieldConstants.TRENCH_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Red TRENCH right
            addRectangularObstacle(
                FieldConstants.TRENCH_WIDTH_METERS,
                FieldConstants.TRENCH_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_TRENCH_RIGHT_X_METERS + FieldConstants.TRENCH_WIDTH_METERS/2,
                    FieldConstants.RED_TRENCH_RIGHT_Y_METERS + FieldConstants.TRENCH_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
        }
        
        /**
         * Adds HUB obstacles.
         */
        private void addHubObstacles() {
            // Blue HUB
            addRectangularObstacle(
                FieldConstants.HUB_WIDTH_METERS,
                FieldConstants.HUB_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_HUB_X_METERS,
                    FieldConstants.BLUE_HUB_Y_METERS,
                    new Rotation2d()
                )
            );
            
            // Red HUB
            addRectangularObstacle(
                FieldConstants.HUB_WIDTH_METERS,
                FieldConstants.HUB_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_HUB_X_METERS,
                    FieldConstants.RED_HUB_Y_METERS,
                    new Rotation2d()
                )
            );
        }
        
        /**
         * Adds DEPOT obstacles.
         */
        private void addDepotObstacles() {
            // Blue DEPOT
            addRectangularObstacle(
                FieldConstants.DEPOT_WIDTH_METERS,
                FieldConstants.DEPOT_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_DEPOT_X_METERS + FieldConstants.DEPOT_WIDTH_METERS/2,
                    FieldConstants.BLUE_DEPOT_Y_METERS + FieldConstants.DEPOT_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Red DEPOT
            addRectangularObstacle(
                FieldConstants.DEPOT_WIDTH_METERS,
                FieldConstants.DEPOT_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_DEPOT_X_METERS + FieldConstants.DEPOT_WIDTH_METERS/2,
                    FieldConstants.RED_DEPOT_Y_METERS + FieldConstants.DEPOT_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
        }
        
        /**
         * Adds TOWER obstacles.
         */
        private void addTowerObstacles() {
            // Blue TOWER
            addRectangularObstacle(
                FieldConstants.TOWER_WIDTH_METERS,
                FieldConstants.TOWER_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_TOWER_X_METERS + FieldConstants.TOWER_DEPTH_METERS/2,
                    FieldConstants.BLUE_TOWER_Y_METERS,
                    new Rotation2d()
                )
            );
            
            // Red TOWER
            addRectangularObstacle(
                FieldConstants.TOWER_WIDTH_METERS,
                FieldConstants.TOWER_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_TOWER_X_METERS - FieldConstants.TOWER_DEPTH_METERS/2,
                    FieldConstants.RED_TOWER_Y_METERS,
                    new Rotation2d()
                )
            );
        }
        
        /**
         * Adds OUTPOST obstacles.
         */
        private void addOutpostObstacles() {
            // Blue OUTPOST (CORRAL area)
            addRectangularObstacle(
                FieldConstants.OUTPOST_CORRAL_WIDTH_METERS,
                FieldConstants.OUTPOST_CORRAL_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.BLUE_OUTPOST_X_METERS + FieldConstants.OUTPOST_CORRAL_WIDTH_METERS/2,
                    FieldConstants.BLUE_OUTPOST_Y_METERS + FieldConstants.OUTPOST_CORRAL_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
            
            // Red OUTPOST (CORRAL area)
            addRectangularObstacle(
                FieldConstants.OUTPOST_CORRAL_WIDTH_METERS,
                FieldConstants.OUTPOST_CORRAL_DEPTH_METERS,
                new Pose2d(
                    FieldConstants.RED_OUTPOST_X_METERS + FieldConstants.OUTPOST_CORRAL_WIDTH_METERS/2,
                    FieldConstants.RED_OUTPOST_Y_METERS + FieldConstants.OUTPOST_CORRAL_DEPTH_METERS/2,
                    new Rotation2d()
                )
            );
        }
    }
}