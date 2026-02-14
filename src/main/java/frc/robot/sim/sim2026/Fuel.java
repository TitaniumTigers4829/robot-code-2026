package frc.robot.sim.sim2026;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.math.forces.Velocity2d;
import frc.robot.extras.math.forces.Velocity3d;
import frc.robot.extras.math.forces.ProjectileUtil.ProjectileDynamics;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimGamePiece;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;

/**
 * Represents a FUEL game piece in the 2026 REBUILT game.
 * 
 * <p>FUEL is a 5.91" diameter foam ball that can be:
 * <ul>
 *   <li>Scored in the HUB (worth 1 point when HUB is active)</li>
 *   <li>Staged in DEPOTs and OUTPOST CHUTEs</li>
 *   <li>Pre-loaded in robots (up to 8 per robot)</li>
 *   <li>Thrown by human players</li>
 * </ul>
 */
public class Fuel extends SimGamePiece {
    
    // Mass in kg (converted from 0.448-0.5 lbs)
    private static final double FUEL_MASS_KG = ((0.448 + 0.5) / 2.0) * 0.453592;
    
    // Diameter in meters (converted from 5.91 inches)
    private static final double FUEL_DIAMETER_M = FieldConstants.FUEL_DIAMETER_METERS;
    
    // Radius in meters
    private static final double FUEL_RADIUS_M = FUEL_DIAMETER_M / 2.0;
    
    /**
     * Creates a new FUEL game piece.
     *
     * @param arena The simulation arena this fuel belongs to
     */
    public Fuel(SimArena arena) {
        super(createFuelVariant(), arena);
    }
    
    /**
     * Creates a new FUEL game piece at a specific position on the field.
     *
     * @param arena The simulation arena this fuel belongs to
     * @param position The initial position on the field
     */
    public Fuel(SimArena arena, Translation2d position) {
        super(createFuelVariant(), arena);
        this.place(position);
    }
    
    /**
     * Creates a new FUEL game piece with initial velocity.
     *
     * @param arena The simulation arena this fuel belongs to
     * @param initialPosition The initial position on the field
     * @param initialVelocity The initial velocity
     */
    public Fuel(SimArena arena, Translation2d initialPosition, Velocity2d initialVelocity) {
        super(createFuelVariant(), arena);
        this.slide(initialPosition, initialVelocity);
    }
    
    /**
     * Creates a new FUEL game piece in flight.
     *
     * @param arena The simulation arena this fuel belongs to
     * @param initialPose The initial pose (position + rotation)
     * @param initialVelocity The initial velocity
     * @param dynamics The projectile dynamics
     */
    public Fuel(SimArena arena, Translation3d initialPose, Velocity3d initialVelocity, 
                   ProjectileDynamics dynamics) {
        super(createFuelVariant(), arena);
        this.launch(new Pose3d(initialPose, new Rotation3d()), initialVelocity, dynamics);
    }
    
    /**
     * Creates the game piece variant for FUEL.
     */
    private static GamePieceVariant createFuelVariant() {
        return new GamePieceVariant(
            "FUEL_2026",
            FUEL_DIAMETER_M, // height (sphere diameter)
            FUEL_MASS_KG,
            Geometry.createCircle(FUEL_RADIUS_M),
            createFuelTargets(),
            true, // place on field when touch ground
            0.3   // landing dampening
        );
    }
    
    /**
     * Creates scoring targets for FUEL in the HUBs.
     * FUEL is scored when it passes through the top opening of a HUB.
     */
    private static List<GamePieceTarget> createFuelTargets() {
        List<GamePieceTarget> targets = new ArrayList<>();
        
        // Blue HUB target (top opening)
        targets.add(new GamePieceTarget(
            new Translation3d(
                FieldConstants.BLUE_HUB_X_METERS - FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.BLUE_HUB_Y_METERS - FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.BLUE_HUB_Z_METERS - 0.1),
            new Translation3d(
                FieldConstants.BLUE_HUB_X_METERS + FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.BLUE_HUB_Y_METERS + FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.BLUE_HUB_Z_METERS + 0.5)
        ));
        
        // Red HUB target (top opening)
        targets.add(new GamePieceTarget(
            new Translation3d(
                FieldConstants.RED_HUB_X_METERS - FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.RED_HUB_Y_METERS - FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.RED_HUB_Z_METERS - 0.1),
            new Translation3d(
                FieldConstants.RED_HUB_X_METERS + FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.RED_HUB_Y_METERS + FieldConstants.HUB_OPENING_SIZE_METERS/2,
                FieldConstants.RED_HUB_Z_METERS + 0.5)
        ));
        
        return targets;
    }
    
    /**
     * @return true if this fuel is currently in a scoring position (in a HUB)
     */
    public boolean isScored() {
        return isInState(GamePieceState.LIMBO) && !isUserControlled();
    }
    
    /**
     * @return true if this fuel is on the field and available for intake
     */
    public boolean isOnField() {
        return isInState(GamePieceState.ON_FIELD) && isLibraryControlled();
    }
    
    /**
     * @return true if this fuel is being held by a robot
     */
    public boolean isHeld() {
        return isInState(GamePieceState.HELD);
    }
    
    /**
     * @return true if this fuel is in flight
     */
    public boolean isInFlight() {
        return isInState(GamePieceState.IN_FLIGHT);
    }
    
    /**
     * @return The diameter of the fuel in meters
     */
    public static double getDiameterMeters() {
        return FUEL_DIAMETER_M;
    }
    
    /**
     * @return The diameter of the fuel in inches
     */
    public static double getDiameterInches() {
        return FieldConstants.FUEL_DIAMETER_INCHES;
    }
    
    /**
     * @return The mass of the fuel in kg
     */
    public static double getMassKg() {
        return FUEL_MASS_KG;
    }
}