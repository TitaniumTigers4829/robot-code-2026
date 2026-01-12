package frc.robot.sim.sim2026;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.sim.simField.SimArena;
import frc.robot.sim.simField.SimArena.FieldMap;
import frc.robot.sim.simField.SimGamePiece.GamePieceTarget;
import frc.robot.sim.simField.SimGamePiece.GamePieceVariant;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.dyn4j.geometry.Geometry;

public class RebuiltSim {
  // private static List<GamePieceTarget> createCoralTargets() {
    // List<Translation3d> reefPositions =
    //     Arrays.asList(
    //         new Translation3d(
    //             Constants.FieldConstants.BLUE_REEF_ONE.getX(),
    //             Constants.FieldConstants.BLUE_REEF_ONE.getY(),
    //             0.0),
    // Add rebuilt game peieces using this syntax

    

//     return reefPositions.stream()
//         .flatMap(
//             position ->
//                 zLevels.stream()
//                     .map(
//                         z ->
//                             new GamePieceTarget(
//                                 new Translation3d(position.getX(), position.getY(), z),
//                                 new Translation3d(position.getX(), position.getY(), z))))
//         .collect(Collectors.toList());
//   }

 
 
  /**
   *
   *
   * <h1>The playing field for the 2025 FRC Game: Reefscape</h1>
   *
   * <p>This class represents the playing field for the 2025 FRC game, Reefscape.
   *
   * <p>It extends {@link SimulatedArena} and includes specific details of the Reefscape game
   * environment.
   */
  }
  // public static class RebuiltSimArena extends SimArena {
  //   public static final class RebuiltFieldObstacleMap extends FieldMap {
  //     public RebuiltFieldObstacleMap() {
  //       super();

  //       // blue wall
  //       addBorderLine(new Translation2d(0, 1.270), new Translation2d(0, 6.782));

  //       // red wall
  //       addBorderLine(new Translation2d(17.548, 1.270), new Translation2d(17.548, 6.782));

  //       // upper walls
  //       addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(17.548 - 1.672, 8.052));

  //       // lower walls
  //       addBorderLine(new Translation2d(1.672, 0), new Translation2d(17.548 - 1.672, 0));

  //     }

  //   public RebuiltSimArena(Time period, int simulationSubTick) {
  //     super(new RebuiltFieldObstacleMap(), period.in(Seconds), simulationSubTick);
  //   }



//     @Override
//     protected void competitionPeriodic() {
//       if (!DriverStation.isTeleopEnabled()) return;

//       if (Timer.getFPGATimestamp() - previousThrowTimeSeconds < 1) return;

    
//       previousThrowTimeSeconds = Timer.getFPGATimestamp();
//     }
//   }
// }
