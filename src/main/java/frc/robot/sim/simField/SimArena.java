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

public abstract class SimArena {
  public record SimEnvTiming(Time period, int ticksPerPeriod, Time dt)
      implements StructSerializable {
    private SimEnvTiming(double robotPeriodSeconds, int simulationSubTicksPerPeriod) {
      this(
          Seconds.of(robotPeriodSeconds),
          simulationSubTicksPerPeriod,
          Seconds.of(robotPeriodSeconds / ((double) simulationSubTicksPerPeriod)));
    }

    public static final Struct<SimEnvTiming> struct = StructGenerator.genRecord(SimEnvTiming.class);
  }

  public final ReentrantLock worldLock = new ReentrantLock();
  public final World<Body> physicsWorld = new World<>();
  public final Set<SimGamePiece> gamePieces = ConcurrentHashMap.newKeySet();
  public final Set<SimRobot<?>> robots = ConcurrentHashMap.newKeySet();
  public final SimEnvTiming timing;
  protected final FieldMap obstaclesMap; // Store the obstacle map

  protected SimArena(FieldMap obstaclesMap, double period, int ticksPerPeriod) {
    this.obstaclesMap = obstaclesMap;
    this.timing = new SimEnvTiming(period, ticksPerPeriod);
    this.physicsWorld.setGravity(World.ZERO_GRAVITY);
    for (FrcBody obstacle : obstaclesMap.obstacles) {
      this.physicsWorld.addBody(obstacle);
    }
  }

  public static final class ReefscapeFieldObstacleMap extends FieldMap {
    public ReefscapeFieldObstacleMap() {}
  }

  public SimArena(Time period, int simulationSubTick) {
    this(new ReefscapeFieldObstacleMap(), period.in(Seconds), simulationSubTick);
  }

  public void withWorld(Consumer<World<Body>> worldModifier) {
    try {
      worldLock.lock();
      worldModifier.accept(physicsWorld);
    } finally {
      worldLock.unlock();
    }
  }

  public SimGamePiece createGamePiece(GamePieceVariant variant) {
    RuntimeLog.debug("Creating a game piece of variant " + variant);
    SimGamePiece gamePiece = new SimGamePiece(variant, this);
    this.gamePieces.add(gamePiece);
    return gamePiece.userControlled();
  }

  public void simulationPeriodic() {
    competitionPeriodic();
    for (int i = 0; i < timing.ticksPerPeriod; i++) {
      simulationSubTick();
    }
  }

  private void simulationSubTick() {
    for (final SimRobot<?> robot : robots) robot.simTick();
    for (final SimGamePiece gamePiece : gamePieces) gamePiece.simulationSubTick();

    withWorld(world -> world.updatev(timing.dt.in(Seconds)));

    // Process collisions after physics update
    processCollisions();
  }

  /** Process collisions using world.isInContact() for each robot. */
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
          // Need to get the underlying body from the piece's state
          // This depends on your SimGamePiece implementation; adjust accordingly
          // For simplicity, assume we can get the body via a method.
          // If not, you may need to modify SimGamePiece to expose its body.
        }
      }
    }

    // Log collision data
    Logger.recordOutput("Collisions/TotalRobots", robots.size());
  }

  public Stream<SimGamePiece> gamePieces() {
    return List.copyOf(gamePieces).stream();
  }

  public void resetFieldForAuto() {
    gamePieces.forEach(gp -> gp.withLib(SimGamePiece::delete));
    gamePieces.clear();
    placeGamePiecesOnField();
  }

  protected abstract void placeGamePiecesOnField();

  protected abstract void competitionPeriodic();

  public static class FieldMap {
    private final List<FrcBody> obstacles = new ArrayList<>();

    protected void addBorderLine(Translation2d startingPoint, Translation2d endingPoint) {
      addCustomObstacle(
          Geometry.createSegment(
              GeomUtil.toDyn4jVector2(startingPoint), GeomUtil.toDyn4jVector2(endingPoint)),
          new Pose2d());
    }

    protected void addRectangularObstacle(
        double width, double height, Pose2d absolutePositionOnField) {
      addCustomObstacle(Geometry.createRectangle(width, height), absolutePositionOnField);
    }

    protected void addCustomObstacle(Convex shape, Pose2d absolutePositionOnField) {
      final FrcBody obstacle = createObstacle(shape);
      obstacle.getTransform().set(GeomUtil.toDyn4jTransform(absolutePositionOnField));
      obstacles.add(obstacle);
    }

    private static FrcBody createObstacle(Convex shape) {
      final FrcBody obstacle = new FrcBody();
      obstacle.setMass(MassType.INFINITE);
      final BodyFixture fixture = obstacle.addFixture(shape);
      fixture.setFriction(0.6);
      fixture.setRestitution(0.3);
      return obstacle;
    }

    public List<FrcBody> getObstacles() {
      return obstacles;
    }
  }
}
