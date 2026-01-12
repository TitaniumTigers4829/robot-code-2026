package frc.robot.sim.simMechanism;

import frc.robot.sim.simField.SimGamePiece.GamePieceCollisionBody;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.dyn4j.collision.CollisionBody;
import org.dyn4j.collision.Fixture;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.geometry.Convex;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.listener.ContactListener;

/**
 * Represents an intake in the sim environment. This intake has collision space which can interact
 * with simulated game pieces and the rest of the simulated environment and can collect game pieces
 * using a {@link SimIndexer}.
 */
public class SimIntake {
  private final BodyFixture fixture;
  private final SimDriveTrain driveTrainSimulation;
  private final List<GamePieceVariant> acceptedGamePieceVariants;
  private final SimIndexer gamePieceStorage;

  private AtomicBoolean intakeRunning = new AtomicBoolean(false);

  /**
   * Creates an Intake Simulation with a Specific Shape.
   *
   * <p>This constructor initializes an intake with a custom shape that is used when the intake is
   * fully extended.
   *
   * @param driveTrainSimulation the chassis to which this intake is attached
   * @param gamePieceStorage the storage for game pieces collected by the intake
   * @param shape the shape of the intake when fully extended, represented as a {@link Convex}
   *     object
   * @param GamePieceVariant the accepted game piece variants for this intake
   */
  public SimIntake(
      SimDriveTrain driveTrainSimulation,
      SimIndexer gamePieceStorage,
      Convex shape,
      GamePieceVariant... acceptedGamePieceVariants) {
    this.fixture = new BodyFixture(shape);
    this.acceptedGamePieceVariants = List.of(acceptedGamePieceVariants);
    this.driveTrainSimulation = driveTrainSimulation;
    this.gamePieceStorage = gamePieceStorage;
  }

  /**
   *
   *
   * <h2>Turns the Intake On.</h2>
   *
   * <p>Extends the intake out from the chassis, making it part of the chassis's collision space.
   *
   * <p>Once activated, the intake is considered running and will listen for contact with {@link
   * GamePieceOnFieldSimulation} instances, allowing it to collect game pieces.
   */
  public void startIntake() {
    if (intakeRunning.getAndSet(true)) {
      return;
    }
    driveTrainSimulation.chassis.addFixture(fixture);
  }

  /**
   *
   *
   * <h2>Turns the Intake Off.</h2>
   *
   * <p>Retracts the intake into the chassis, removing it from the chassis's collision space.
   *
   * <p>Once turned off, the intake will no longer listen for or respond to contacts with {@link
   * GamePieceOnFieldSimulation} instances.
   */
  public void stopIntake() {
    if (!intakeRunning.getAndSet(false)) {
      return;
    }
    driveTrainSimulation.chassis.removeFixture(fixture);
  }

  /**
   *
   *
   * <h2>The {@link ContactListener} for the Intake Simulation.</h2>
   *
   * <p>This class can be added to the simulation world to detect and manage contacts between the
   * intake and {@link GamePieceOnFieldSimulation} instances of the type {@link
   * #targetedGamePieceType}.
   *
   * <p>If contact is detected and the intake is running, the {@link GamePieceOnFieldSimulation}
   * will be marked for removal from the field.
   */
  final class GamePieceContactListener implements ContactListener<Body> {
    @Override
    public void begin(ContactCollisionData<Body> collision, Contact contact) {
      if (!SimIntake.this.intakeRunning.get()) return;

      final CollisionBody<?> collisionBody1 = collision.getBody1();
      final CollisionBody<?> collisionBody2 = collision.getBody2();
      final Fixture fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();

      if (collisionBody1 instanceof GamePieceCollisionBody gamePiece
          && acceptedGamePieceVariants.contains(gamePiece.gp.variant())
          && fixture2 == SimIntake.this.fixture) {
        intakeGamePiece(gamePiece);
      } else if (collisionBody2 instanceof GamePieceCollisionBody gamePiece
          && acceptedGamePieceVariants.contains(gamePiece.gp.variant())
          && fixture1 == SimIntake.this.fixture) {
        intakeGamePiece(gamePiece);
      }
    }

    private void intakeGamePiece(GamePieceCollisionBody gamePieceBody) {
      SimIntake sim = SimIntake.this;
      sim.gamePieceStorage.insertGamePiece(gamePieceBody.gp);
    }

    /* functions not used */
    @Override
    public void persist(
        ContactCollisionData<Body> collision, Contact oldContact, Contact newContact) {}

    @Override
    public void end(ContactCollisionData<Body> collision, Contact contact) {}

    @Override
    public void destroyed(ContactCollisionData<Body> collision, Contact contact) {}

    @Override
    public void collision(ContactCollisionData<Body> collision) {}

    @Override
    public void preSolve(ContactCollisionData<Body> collision, Contact contact) {}

    @Override
    public void postSolve(ContactCollisionData<Body> collision, SolvedContact contact) {}
  }

  public GamePieceContactListener getGamePieceContactListener() {
    return new GamePieceContactListener();
  }
}
