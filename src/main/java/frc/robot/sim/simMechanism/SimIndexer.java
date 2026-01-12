package frc.robot.sim.simMechanism;

import frc.robot.sim.simField.SimGamePiece;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;

/** Represents an indexer in the sim environment. An indexer can store game pieces. */
public class SimIndexer {
  protected final Queue<SimGamePiece> gamePieces;
  protected final Set<GamePieceVariant> gamePiecesSet;
  protected final int numPieces;

  /**
   * Creates a new GamePieceStorage that can store the given number of pieces.
   *
   * @param numPieces the number of pieces to store
   * @param gamePieceVariantFilet the variants of game pieces to store, or an empty array to store
   *     all variants
   */
  public SimIndexer(int numPieces, GamePieceVariant... gamePieceVariantFilet) {
    gamePieces = new LinkedList<>();
    this.numPieces = numPieces;
    gamePiecesSet = Set.of(gamePieceVariantFilet);
  }

  /** Inserts a game piece into the storage, regardless of the storage's current state. */
  boolean forceInsertGamePiece(SimGamePiece gamePiece) {
    if (gamePieces.size() < numPieces
        && (gamePiecesSet.contains(gamePiece.variant()) || gamePiecesSet.isEmpty())) {
      gamePiece.releaseControl();
      gamePiece.withLib(gp -> gp.intake());
      gamePieces.add(gamePiece);
      return true;
    } else {
      return false;
    }
  }

  /**
   * Inserts a game piece into the storage.
   *
   * <p>Insertion will fail if the game piece is library-controlled, the storage is full or the game
   * piece is not of the correct variant.
   *
   * @param gamePiece the game piece to insert
   * @return if the game piece was successfully inserted
   */
  public boolean insertGamePiece(SimGamePiece gamePiece) {
    if (gamePiece.isLibraryControlled()) {
      return false;
    }
    return forceInsertGamePiece(gamePiece);
  }

  /**
   * Removes a game piece from the storage and returns it if a game piece is present.
   *
   * @return the game piece that was removed, or an empty optional if no game piece was present
   */
  public Optional<SimGamePiece> removeGamePiece() {
    return Optional.ofNullable(gamePieces.poll()).map(SimGamePiece::userControlled);
  }

  /**
   * Removes all game pieces from the storage.
   *
   * <p>This method also transitions all game pieces to limbo state.
   */
  public void clear() {
    for (var gamePiece : gamePieces) {
      gamePiece.delete();
    }
    gamePieces.clear();
  }
}
