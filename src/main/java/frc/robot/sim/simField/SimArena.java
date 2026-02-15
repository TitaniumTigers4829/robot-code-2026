package frc.robot.sim.simField;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.extras.logging.RuntimeLog;
import frc.robot.extras.math.mathutils.GeomUtil;
import frc.robot.extras.util.FrcBody;
import frc.robot.sim.SimRobot;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.World;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for a simulation arena.
 *
 * <p>This class manages the physics world, game pieces, and robots in a simulation environment.
 * It handles:
 * <ul>
 *   <li>Physics world initialization and updates</li>
 *   <li>Collision detection between robots, obstacles, and game pieces</li>
 *   <li>Game piece creation and management</li>
 *   <li>Robot registration and simulation ticks</li>
 * </ul>
 *
 * <p>Season-specific implementations (e.g., {@link frc.robot.sim.sim2026.RebuiltSim.RebuiltSimArena})
 * should extend this class and implement {@link #placeGamePiecesOnField()} and
 * {@link #competitionPeriodic()} to provide game-specific behavior.
 *
 * @see SimGamePiece
 * @see SimRobot
 * @see FieldMap
 */
public abstract class SimArena {

  /**
   * Timing information for the simulation environment.
   *
   * @param period The robot periodic loop time
   * @param ticksPerPeriod Number of physics sub-ticks per robot period
   * @param dt Duration of each physics sub-tick
   */
  public record SimEnvTiming(Time period, int ticksPerPeriod, Time dt)
      implements StructSerializable {
    
    /**
     * Creates timing information from robot period and sub-ticks.
     *
     * @param robotPeriodSeconds The robot periodic loop time in seconds
     * @param simulationSubTicksPerPeriod Number of physics sub-ticks per robot period
     */
    private SimEnvTiming(double robotPeriodSeconds, int simulationSubTicksPerPeriod) {
      this(
          Seconds.of(robotPeriodSeconds),
          simulationSubTicksPerPeriod,
          Seconds.of(robotPeriodSeconds / ((double) simulationSubTicksPerPeriod)));
    }

    /** Struct for serialization of SimEnvTiming. */
    public static final Struct<SimEnvTiming> struct = StructGenerator.genRecord(SimEnvTiming.class);
  }

  /** Lock for thread-safe access to the physics world. */
  public final ReentrantLock worldLock = new ReentrantLock();

  /** The dyn4j physics world containing all bodies (robots, obstacles, game pieces). */
  public final World<Body> physicsWorld = new World<>();

  /** Set of all game pieces currently in the simulation. */
  public final Set<SimGamePiece> gamePieces = ConcurrentHashMap.newKeySet();

  /** Set of all robots currently in the simulation. */
  public final Set<SimRobot<?>> robots = ConcurrentHashMap.newKeySet();

  /** Timing information for the simulation. */
  public final SimEnvTiming timing;

  /** The field obstacle map containing all static obstacles. */
  protected final FieldMap obstaclesMap;

  /**
   * Constructs a new simulation arena with the specified field map of obstacles.
   *
   * <p>This constructor initializes a physics world with zero gravity and adds the provided
   * obstacles to the world.
   *
   * @param obstaclesMap the season-specific field map containing the layout of obstacles
   * @param period the duration of each simulation period in seconds (typically robot loop time)
   * @param ticksPerPeriod the number of physics sub-ticks to execute in each simulation period
   */
  protected SimArena(FieldMap obstaclesMap, double period, int ticksPerPeriod) {
    this.obstaclesMap = obstaclesMap;
    this.timing = new SimEnvTiming(period, ticksPerPeriod);
    this.physicsWorld.setGravity(World.ZERO_GRAVITY);
    for (FrcBody obstacle : obstaclesMap.obstacles) {
      this.physicsWorld.addBody(obstacle);
    }
  }

  /**
   * Default field obstacle map for the Reefscape game (2025).
   *
   * @deprecated Use season-specific implementations instead.
   */
  @Deprecated
  public static final class ReefscapeFieldObstacleMap extends FieldMap {
    public ReefscapeFieldObstacleMap() {}
  }

  /**
   * Constructs a new simulation arena with a default Reefscape field map.
   *
   * @param period the simulation period
   * @param simulationSubTick the number of physics sub-ticks per period
   * @deprecated Use season-specific constructors instead.
   */
  @Deprecated
  public SimArena(Time period, int simulationSubTick) {
    this(new ReefscapeFieldObstacleMap(), period.in(Seconds), simulationSubTick);
  }

  /**
   * Executes an operation with the physics world locked.
   *
   * <p>This method ensures thread-safe access to the physics world by acquiring
   * {@link #worldLock} before executing the provided consumer.
   *
   * @param worldModifier operation to perform on the locked world
   */
  public void withWorld(Consumer<World<Body>> worldModifier) {
    try {
      worldLock.lock();
      worldModifier.accept(physicsWorld);
    } finally {
      worldLock.unlock();
    }
  }

  /**
   * Creates and registers a new game piece in the simulation.
   *
   * <p>The created game piece is initially in user-controlled state. The caller must
   * call appropriate methods (e.g., {@link SimGamePiece#place(Translation2d)}) to
   * add it to the field.
   *
   * @param variant the variant of game piece to create
   * @return the created game piece (in user-controlled state)
   */
  public SimGamePiece createGamePiece(GamePieceVariant variant) {
    RuntimeLog.debug("Creating a game piece of variant " + variant);
    SimGamePiece gamePiece = new SimGamePiece(variant, this);
    this.gamePieces.add(gamePiece);
    return gamePiece.userControlled();
  }

  /**
   * Updates the simulation world.
   *
   * <p>This method should be called ONCE in {@link edu.wpi.first.wpilibj.IterativeRobotBase#simulationPeriodic()}.
   * It executes:
   * <ol>
   *   <li>Game-specific competition logic via {@link #competitionPeriodic()}</li>
   *   <li>Multiple physics sub-ticks (as specified by {@link SimEnvTiming#ticksPerPeriod})</li>
   * </ol>
   */
  public void simulationPeriodic() {
    competitionPeriodic();
    for (int i = 0; i < timing.ticksPerPeriod; i++) {
      simulationSubTick();
    }
  }

  /**
   * Executes a single physics sub-tick.
   *
   * <p>This method:
   * <ol>
   *   <li>Updates all robots via {@link SimRobot#simTick()}</li>
   *   <li>Updates all game pieces via {@link SimGamePiece#simulationSubTick()}</li>
   *   <li>Advances the physics world by one time step ({@link SimEnvTiming#dt})</li>
   *   <li>Processes collisions between robots and obstacles</li>
   * </ol>
   */
  private void simulationSubTick() {
    for (final SimRobot<?> robot : robots) robot.simTick();
    for (final SimGamePiece gamePiece : gamePieces) gamePiece.simulationSubTick();

    withWorld(world -> world.updatev(timing.dt.in(Seconds)));

    // Process collisions after physics update
    processCollisions();
  }

  /**
   * Processes collisions between robots and field obstacles.
   *
   * <p>This method:
   * <ul>
   *   <li>Clears previous collision data from all robots</li>
   *   <li>Checks each robot against all obstacles using {@link World#isInContact(Body, Body)}</li>
   *   <li>Checks each robot against other robots</li>
   *   <li>Updates each robot's collision list via {@link FrcBody#addCollidingBody(Body)}</li>
   *   <li>Logs collision statistics</li>
   * </ul>
   *
   * <p>Note: Game piece collision detection is currently a placeholder and should be
   * implemented by accessing the underlying body from {@link SimGamePiece} when in
   * {@link SimGamePiece.GamePieceState#ON_FIELD} state.
   */
  private void processCollisions() {
    // Clear previous collision data for all robots
    for (SimRobot<?> robot : robots) {
      robot.getDriveTrain().chassis.clearCollisions();
    }

    // Get all obstacles
    List<FrcBody> obstacles = obstaclesMap.getObstacles();

    // Check each robot against obstacles, other robots, and game pieces
    for (SimRobot<?> robot : robots) {
      FrcBody robotBody = robot.getDriveTrain().chassis;

      // Check against obstacles
      for (FrcBody obstacle : obstacles) {
        if (physicsWorld.isInContact(robotBody, obstacle)) {
          robotBody.addCollidingBody(obstacle);
        }
      }

      // Check against other robots
      for (SimRobot<?> other : robots) {
        if (other == robot) continue;
        FrcBody otherBody = other.getDriveTrain().chassis;
        if (physicsWorld.isInContact(robotBody, otherBody)) {
          robotBody.addCollidingBody(otherBody);
        }
      }

      // Check against game pieces on the field
      for (SimGamePiece piece : gamePieces) {
        if (piece.state() == SimGamePiece.GamePieceState.ON_FIELD) {
          // TODO: Implement game piece collision detection
          // This requires accessing the underlying body from the piece's state
        }
      }
    }

    // Log collision data
    Logger.recordOutput("Collisions/TotalRobots", robots.size());
  }

  /**
   * Returns a stream of all game pieces tracked by the arena.
   *
   * <p>Useful for filtering and collecting game pieces by state or variant.
   *
   * <pre><code>
   * // Example: Get positions of all FUEL on the field
   * Pose3d[] fuelPositions = arena.gamePieces()
   *     .filter(GamePieceState.ON_FIELD::isInState)
   *     .filter(Fuel.VARIANT::isOfVariant)
   *     .collect(SimGamePiece.poseStreamCollector());
   * </code></pre>
   *
   * @return stream of game pieces
   */
  public Stream<SimGamePiece> gamePieces() {
    return List.copyOf(gamePieces).stream();
  }

  /**
   * Resets the field for autonomous mode.
   *
   * <p>This method clears all existing game pieces and places new ones
   * in their starting positions via {@link #placeGamePiecesOnField()}.
   */
  public void resetFieldForAuto() {
    gamePieces.forEach(gp -> gp.withLib(SimGamePiece::delete));
    gamePieces.clear();
    placeGamePiecesOnField();
  }

  /**
   * Places game pieces on the field for autonomous mode.
   *
   * <p>This method should be implemented by season-specific subclasses to place
   * game pieces in their correct starting positions according to the game manual.
   */
  protected abstract void placeGamePiecesOnField();

  /**
   * Executes season-specific competition logic each simulation period.
   *
   * <p>Implementations should handle:
   * <ul>
   *   <li>Score tracking and updates</li>
   *   <li>Game element state changes (e.g., HUB active/inactive)</li>
   *   <li>Human player interactions</li>
   *   <li>Match timing events</li>
   * </ul>
   */
  protected abstract void competitionPeriodic();

  /**
   * Abstract field map containing static obstacles.
   *
   * <p>This class provides methods for adding obstacles to the field:
   * <ul>
   *   <li>Border lines (walls)</li>
   *   <li>Rectangular obstacles (BUMPS, TRENCHES, etc.)</li>
   *   <li>Custom convex shapes</li>
   * </ul>
   *
   * <p>Obstacles are created with infinite mass (static) and appropriate
   * friction and restitution coefficients.
   */
  public static class FieldMap {
    /** List of static obstacles in the field. */
    private final List<FrcBody> obstacles = new ArrayList<>();

    /**
     * Adds a border line (wall) between two points.
     *
     * @param startingPoint start of the line
     * @param endingPoint end of the line
     */
    protected void addBorderLine(Translation2d startingPoint, Translation2d endingPoint) {
      addCustomObstacle(
          Geometry.createSegment(
              GeomUtil.toDyn4jVector2(startingPoint), GeomUtil.toDyn4jVector2(endingPoint)),
          new Pose2d());
    }

    /**
     * Adds a rectangular obstacle at the specified position.
     *
     * @param width width of the rectangle in meters
     * @param height height of the rectangle in meters
     * @param absolutePositionOnField pose of the rectangle center
     */
    protected void addRectangularObstacle(
        double width, double height, Pose2d absolutePositionOnField) {
      addCustomObstacle(Geometry.createRectangle(width, height), absolutePositionOnField);
    }

    /**
     * Adds a custom convex obstacle at the specified position.
     *
     * @param shape the convex shape of the obstacle
     * @param absolutePositionOnField pose of the shape
     */
    protected void addCustomObstacle(Convex shape, Pose2d absolutePositionOnField) {
      final FrcBody obstacle = createObstacle(shape);
      obstacle.getTransform().set(GeomUtil.toDyn4jTransform(absolutePositionOnField));
      obstacles.add(obstacle);
    }

    /**
     * Creates a static obstacle body with the given shape.
     *
     * <p>The obstacle is configured with:
     * <ul>
     *   <li>Infinite mass (static)</li>
     *   <li>Friction coefficient: 0.6</li>
     *   <li>Restitution coefficient: 0.3</li>
     * </ul>
     *
     * @param shape the convex shape of the obstacle
     * @return the created obstacle body
     */
    private static FrcBody createObstacle(Convex shape) {
      final FrcBody obstacle = new FrcBody();
      obstacle.setMass(MassType.INFINITE);
      final BodyFixture fixture = obstacle.addFixture(shape);
      fixture.setFriction(0.6);
      fixture.setRestitution(0.3);
      return obstacle;
    }

    /**
     * Gets the list of all obstacles in this field map.
     *
     * @return list of obstacle bodies
     */
    public List<FrcBody> getObstacles() {
      return obstacles;
    }
  }
}