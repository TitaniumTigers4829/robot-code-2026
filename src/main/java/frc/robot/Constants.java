// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static RobotType robotType = RobotType.SIM_ROBOT;

  public static final boolean tuningMode = false;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case DEV_ROBOT, COMP_ROBOT, SWERVE_ROBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM_ROBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotType {
    SIM_ROBOT,
    DEV_ROBOT,
    COMP_ROBOT,
    SWERVE_ROBOT
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class HardwareConstants {
    public static final double LOOP_TIME_SECONDS = 0.02;
    public static final double RIO_SIGNAL_FREQUENCY = 50;
    public static final double CANIVORE_SIGNAL_FREQUENCY = 250;
    public static final String CANIVORE_CAN_BUS_STRING = "carnivore 1";
    public static final String RIO_CAN_BUS_STRING = "rio";
    public static final double MIN_DUTY_CYCLE_DEADBAND = 0.001;
    public static final double MIN_TORQUE_DEADBAND = 5.0;
    public static final int HIGH_THREAD_PRIORITY = 99;
    public static final int LOW_THREAD_PRIORITY = 1;
  }

  public static final class FieldConstants {
    // ============================================================
    // COORDINATE SYSTEM NOTE
    // ============================================================
    // WPILib field convention used throughout:
    //   X = 0 at Blue Alliance wall, +X toward Red Alliance wall
    //   Y = 0 at Audience-side wall, +Y toward Scoring Table wall
    //   Z = 0 at carpet, +Z up
    //
    // The official AprilTag table (FE-2026 Sheet 11, Welded Perimeter)
    // uses a DIFFERENT convention:
    //   X = 0 at Blue wall (same)
    //   Y = 0 at Scoring Table wall, +Y toward Audience (FLIPPED)
    //   Z = 0 at carpet (same)
    //
    // Conversion: WPILib_Y = FIELD_WIDTH_INCHES - AprilTag_Y
    //
    // All element positions below are from the Welded Perimeter table
    // unless explicitly noted. AndyMark perimeter values differ by
    // ~0.5–1.1" on most dimensions.
    // ============================================================

    // ===== FIELD DIMENSIONS =====
    // Source: FE-2026 Sheet 3 reference dimensions (651.22), (317.69)
    public static final double FIELD_LENGTH_INCHES = 651.22;
    public static final double FIELD_WIDTH_INCHES = 317.69;

    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(FIELD_LENGTH_INCHES);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(FIELD_WIDTH_INCHES);

    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH_METERS / 2.0, FIELD_WIDTH_METERS / 2.0);

    // Alliance zone depth — from alliance wall to center line
    // Source: FE-2026 Sheet 3 reference dimension (158.84) per side
    public static final double ALLIANCE_ZONE_DEPTH_INCHES = 158.84;
    public static final double ALLIANCE_ZONE_DEPTH_METERS =
        Units.inchesToMeters(ALLIANCE_ZONE_DEPTH_INCHES);

    // Center line X
    public static final double CENTER_LINE_X_INCHES = FIELD_LENGTH_INCHES / 2.0; // 325.61
    public static final double CENTER_LINE_X_METERS = Units.inchesToMeters(CENTER_LINE_X_INCHES);

    // ===== HUB POSITIONS =====
    // Source: FE-2026 Sheet 11 (AprilTag table, AndyMark Perimeter — FIRST North Carolina)
    //
    // AndyMark hub center X: average of Blue-facing tags (X=157.79) and Red-facing (X=205.32):
    //   (157.79 + 205.32) / 2 = 181.555 ≈ 181.56 ✓ (matches Sheet 11 AndyMark ID 20 at X=205.32)
    //
    // HUB Y FACES (used for BUMP placement and scoring target bounds):
    //   - Hub audience-facing side:      WPILib Y = 182.08"   (AT tags 21,24 at Y_at=134.56)
    //   - Hub scoring-table-facing side: WPILib Y = 134.56"   (AT tags 27,18 at Y_at=182.08)
    //   Hub depth (Y span): 182.08 - 134.56 = 47.52" ≈ 47" ✓

    public static final double HUB_FACE_AUDIENCE_SIDE_Y_INCHES = 182.08; // WPILib Y (AndyMark)
    public static final double HUB_FACE_SCORINGTABLE_SIDE_Y_INCHES = 134.56; // WPILib Y (AndyMark)

    public static final double BLUE_HUB_X_INCHES = 181.56; // AndyMark (Welded was 182.11)
    public static final double BLUE_HUB_Y_INCHES = FIELD_WIDTH_INCHES / 2.0; // 158.845"
    public static final double BLUE_HUB_Z_INCHES = 72.0; // top opening height (GE-26300 sheet 4)

    public static final double BLUE_HUB_X_METERS = Units.inchesToMeters(BLUE_HUB_X_INCHES);
    public static final double BLUE_HUB_Y_METERS = Units.inchesToMeters(BLUE_HUB_Y_INCHES);
    public static final double BLUE_HUB_Z_METERS = Units.inchesToMeters(BLUE_HUB_Z_INCHES);

    // Red HUB: FIELD_LENGTH - 181.56 = 469.66" … but Sheet 11 AndyMark gives Red hub tags
    // at X=468.56 (90°/270° face tags), so center X = (444.80+492.33)/2 = 468.565 ≈ 468.56
    public static final double RED_HUB_X_INCHES = 468.56; // AndyMark (direct from Sheet 11)
    public static final double RED_HUB_Y_INCHES = BLUE_HUB_Y_INCHES;
    public static final double RED_HUB_Z_INCHES = BLUE_HUB_Z_INCHES;

    public static final double RED_HUB_X_METERS = Units.inchesToMeters(RED_HUB_X_INCHES);
    public static final double RED_HUB_Y_METERS = Units.inchesToMeters(RED_HUB_Y_INCHES);
    public static final double RED_HUB_Z_METERS = Units.inchesToMeters(RED_HUB_Z_INCHES);

    // HUB dimensions (GE-26300 sheet 4 of 4)
    // Outside dimension: 47.00" × 47.00" (3× 41.932 inside hex opening, 47" outside)
    public static final double HUB_WIDTH_INCHES = 47.00; // along X
    public static final double HUB_DEPTH_INCHES = 47.51; // along Y (hub face span from tags)
    public static final double HUB_HEIGHT_INCHES = 72.00; // total height to top opening
    public static final double HUB_WIDTH_METERS = Units.inchesToMeters(HUB_WIDTH_INCHES);
    public static final double HUB_DEPTH_METERS = Units.inchesToMeters(HUB_DEPTH_INCHES);
    public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(HUB_HEIGHT_INCHES);

    // Hex funnel opening size (GE-26300 sheet 4: 3× 41.932 outside dimension hex)
    public static final double HUB_OPENING_SIZE_INCHES = 41.7;
    public static final double HUB_OPENING_SIZE_METERS =
        Units.inchesToMeters(HUB_OPENING_SIZE_INCHES);

    // Convenience Translation2d for sim code
    public static final Translation2d BLUE_HUB_CENTER =
        new Translation2d(BLUE_HUB_X_METERS, BLUE_HUB_Y_METERS);
    public static final Translation2d RED_HUB_CENTER =
        new Translation2d(RED_HUB_X_METERS, RED_HUB_Y_METERS);

    // ===== BUMP POSITIONS =====
    // Source: GE-26100 (BUMP drawing, sheet 2 of 2) for dimensions;
    //         FE-2026 Sheet 3 reference dims for Y placement.
    //
    // CRITICAL FIX: The old code placed BUMPs offset along X from the HUB.
    // BUMPs are actually aligned in X with the HUB and flank it along Y
    // (one bump on the audience side, one on the scoring-table side of each HUB).
    //
    // BUMP footprint from GE-26100:
    //   - Width (X direction, parallel to hub face): 48.93"
    //   - Depth (Y direction, perpendicular toward wall): 44.40"
    //   - Height at peak: 6.56" (15° ramp angle)
    //
    // BUMP Y placement (derived from FE-2026 Sheet 3 reference dim (90.95)):
    //   - Sheet 3 shows the bump outer edge at 90.95" from each side wall.
    //   - Audience-side bump occupies WPILib Y = 90.95" to 135.09" (depth=44.14" ≈ 44.40" ✓)
    //   - Scoring-table-side bump occupies WPILib Y = 182.60" to 226.74" (depth=44.14" ≈ 44.40" ✓)
    //   - Centers at Y ≈ 113.02" and 204.67" respectively.
    //
    // NOTE: BUMP X is centered on HUB X (182.11" Blue, 469.11" Red).
    public static final double BUMP_WIDTH_INCHES = 48.93; // along X (was 73.0, WRONG)
    public static final double BUMP_DEPTH_INCHES = 44.40; // along Y
    public static final double BUMP_HEIGHT_INCHES = 6.56; // peak height
    public static final double BUMP_ANGLE_DEGREES = 15.0;

    public static final double BUMP_WIDTH_METERS = Units.inchesToMeters(BUMP_WIDTH_INCHES);
    public static final double BUMP_DEPTH_METERS = Units.inchesToMeters(BUMP_DEPTH_INCHES);
    public static final double BUMP_HEIGHT_METERS = Units.inchesToMeters(BUMP_HEIGHT_INCHES);

    // Outer Y edges of bumps (from sheet 3 ref dim "90.95" from each wall)
    private static final double BUMP_OUTER_EDGE_FROM_WALL_INCHES = 90.95;

    // Blue BUMP — audience side (low Y, between audience wall and hub)
    // Center = (outer_edge + hub_face) / 2 = (90.95 + 135.09) / 2 = 113.02"
    public static final double BLUE_BUMP_AUDIENCE_CENTER_X_INCHES = BLUE_HUB_X_INCHES; // 182.11
    public static final double BLUE_BUMP_AUDIENCE_CENTER_Y_INCHES = // ~113.02"
        (BUMP_OUTER_EDGE_FROM_WALL_INCHES + HUB_FACE_SCORINGTABLE_SIDE_Y_INCHES) / 2.0;

    // Blue BUMP — scoring table side (high Y, between hub and scoring table wall)
    // Outer edge at 317.69 - 90.95 = 226.74"; center = (182.60 + 226.74) / 2 = 204.67"
    public static final double BLUE_BUMP_SCORINGTABLE_CENTER_X_INCHES = BLUE_HUB_X_INCHES;
    public static final double BLUE_BUMP_SCORINGTABLE_CENTER_Y_INCHES = // ~204.67"
        (HUB_FACE_AUDIENCE_SIDE_Y_INCHES + (FIELD_WIDTH_INCHES - BUMP_OUTER_EDGE_FROM_WALL_INCHES))
            / 2.0;

    // Red BUMPs are symmetric about field center X
    public static final double RED_BUMP_AUDIENCE_CENTER_X_INCHES = RED_HUB_X_INCHES; // 469.11
    public static final double RED_BUMP_AUDIENCE_CENTER_Y_INCHES =
        BLUE_BUMP_AUDIENCE_CENTER_Y_INCHES;
    public static final double RED_BUMP_SCORINGTABLE_CENTER_X_INCHES = RED_HUB_X_INCHES;
    public static final double RED_BUMP_SCORINGTABLE_CENTER_Y_INCHES =
        BLUE_BUMP_SCORINGTABLE_CENTER_Y_INCHES;

    public static final double BLUE_BUMP_AUDIENCE_CENTER_X_METERS =
        Units.inchesToMeters(BLUE_BUMP_AUDIENCE_CENTER_X_INCHES);
    public static final double BLUE_BUMP_AUDIENCE_CENTER_Y_METERS =
        Units.inchesToMeters(BLUE_BUMP_AUDIENCE_CENTER_Y_INCHES);
    public static final double BLUE_BUMP_SCORINGTABLE_CENTER_X_METERS =
        Units.inchesToMeters(BLUE_BUMP_SCORINGTABLE_CENTER_X_INCHES);
    public static final double BLUE_BUMP_SCORINGTABLE_CENTER_Y_METERS =
        Units.inchesToMeters(BLUE_BUMP_SCORINGTABLE_CENTER_Y_INCHES);
    public static final double RED_BUMP_AUDIENCE_CENTER_X_METERS =
        Units.inchesToMeters(RED_BUMP_AUDIENCE_CENTER_X_INCHES);
    public static final double RED_BUMP_AUDIENCE_CENTER_Y_METERS =
        Units.inchesToMeters(RED_BUMP_AUDIENCE_CENTER_Y_INCHES);
    public static final double RED_BUMP_SCORINGTABLE_CENTER_X_METERS =
        Units.inchesToMeters(RED_BUMP_SCORINGTABLE_CENTER_X_INCHES);
    public static final double RED_BUMP_SCORINGTABLE_CENTER_Y_METERS =
        Units.inchesToMeters(RED_BUMP_SCORINGTABLE_CENTER_Y_INCHES);

    // ===== TRENCH POSITIONS =====
    // Source: FE-2026 Sheet 11 AprilTag table (Welded Perimeter)
    //
    // Each alliance has two TRENCH assemblies (one near audience wall, one near
    // scoring-table wall). Each assembly consists of a Hinged + Fixed half
    // (GE-26201, GE-26202). The tag is located at the hub-end of the structure.
    //
    // The TRENCH runs in the X direction from the alliance wall (~X=0) to the
    // hub face (~hub_X), at a fixed Y position near each perimeter wall.
    //
    // Tag Y positions (WPILib, converted from AprilTag table):
    //   Audience-side trench:      WPILib Y = FIELD_WIDTH - 292.31 = 25.38"
    //   Scoring-table-side trench: WPILib Y = FIELD_WIDTH -  25.37 = 292.32"
    //
    // TRENCH clearance height: 22.25" ±0.25" (FE-2026 Sheet 8 controlled dim)
    // TRENCH depth in Y:  ~3" (rail/beam, not modeled as a wide obstacle)
    //   → For the obstacle map, model as a narrow line obstacle in X.
    //
    // NOTE: BLUE_TRENCH_RIGHT_X was previously "FIELD_LENGTH / 4 = 162.8"" — completely wrong.
    // All trenches are centered around the hub X, not the field quarter-length.

    // Y-center of trench rails (same for both alliances — symmetric)
    public static final double TRENCH_AUDIENCE_SIDE_Y_INCHES = 25.38; // WPILib Y
    public static final double TRENCH_SCORINGTABLE_SIDE_Y_INCHES = 292.32; // WPILib Y
    public static final double TRENCH_AUDIENCE_SIDE_Y_METERS =
        Units.inchesToMeters(TRENCH_AUDIENCE_SIDE_Y_INCHES);
    public static final double TRENCH_SCORINGTABLE_SIDE_Y_METERS =
        Units.inchesToMeters(TRENCH_SCORINGTABLE_SIDE_Y_INCHES);

    // Trench clearance (controlled dimension, FE-2026 Sheet 8)
    public static final double TRENCH_CLEARANCE_HEIGHT_INCHES = 22.25;
    public static final double TRENCH_CLEARANCE_HEIGHT_METERS =
        Units.inchesToMeters(TRENCH_CLEARANCE_HEIGHT_INCHES);

    // Blue trench X extents: from Blue wall (0) to Blue hub audience-facing X
    // Tag X positions (from AprilTag table): 180.64" and 183.59" (fixed + hinged ends)
    public static final double BLUE_TRENCH_START_X_INCHES = 0.0; // Blue alliance wall
    public static final double BLUE_TRENCH_END_X_INCHES = BLUE_HUB_X_INCHES; // 182.11 (hub face)
    public static final double BLUE_TRENCH_CENTER_X_INCHES =
        (BLUE_TRENCH_START_X_INCHES + BLUE_TRENCH_END_X_INCHES) / 2.0; // 91.055
    public static final double BLUE_TRENCH_LENGTH_X_INCHES = BLUE_TRENCH_END_X_INCHES; // 182.11"

    public static final double BLUE_TRENCH_CENTER_X_METERS =
        Units.inchesToMeters(BLUE_TRENCH_CENTER_X_INCHES);
    public static final double BLUE_TRENCH_LENGTH_X_METERS =
        Units.inchesToMeters(BLUE_TRENCH_LENGTH_X_INCHES);

    // Red trench X extents: from Red hub face (469.11") to Red wall (651.22")
    public static final double RED_TRENCH_START_X_INCHES = RED_HUB_X_INCHES; // 469.11
    public static final double RED_TRENCH_END_X_INCHES = FIELD_LENGTH_INCHES; // 651.22
    public static final double RED_TRENCH_CENTER_X_INCHES =
        (RED_TRENCH_START_X_INCHES + RED_TRENCH_END_X_INCHES) / 2.0; // 560.165
    public static final double RED_TRENCH_LENGTH_X_INCHES =
        RED_TRENCH_END_X_INCHES - RED_TRENCH_START_X_INCHES; // 182.11"

    public static final double RED_TRENCH_CENTER_X_METERS =
        Units.inchesToMeters(RED_TRENCH_CENTER_X_INCHES);
    public static final double RED_TRENCH_LENGTH_X_METERS =
        Units.inchesToMeters(RED_TRENCH_LENGTH_X_INCHES);

    // ===== TOWER POSITIONS =====
    // Source: FE-2026 Sheet 11 AprilTag table (Welded Perimeter)
    //
    // Blue TOWER — tags ID 31 and ID 32:
    //   AT_31: X=0.32, Y=147.47 → WPILib (0.32, 170.22)
    //   AT_32: X=0.32, Y=164.47 → WPILib (0.32, 153.22)
    //   Center WPILib Y = (170.22 + 153.22) / 2 = 161.72"
    //
    // Red TOWER — tags ID 15 and ID 16:
    //   AT_15: X=650.90, Y=170.22 → WPILib (650.90, 147.47)
    //   AT_16: X=650.90, Y=153.22 → WPILib (650.90, 164.47)
    //   Center WPILib Y = (147.47 + 164.47) / 2 = 155.97"
    //
    // NOTE: Old value was FIELD_WIDTH * 0.4 = 127.1" — completely wrong.
    // Note the slight Blue/Red asymmetry in Y (~5.75") which mirrors the actual field.
    //
    // TOWER rung heights from GE-26500 sheet 2 (controlled dimensions):
    //   Low rung:  27.000 ±0.250"
    //   Mid rung:  45.000 ±0.250"
    //   High rung: 63.000 ±0.250"
    //
    // TOWER footprint from GE-26500 sheet 2:
    //   Width (Y direction): 40.000 ±1.000" (across rungs)
    //   Depth (X direction): 32.250" (base plate depth)
    //   Total height: 72.125"

    public static final double TOWER_WIDTH_INCHES = 40.00; // along Y (across rungs)
    public static final double TOWER_DEPTH_INCHES = 32.25; // along X (into field)
    public static final double TOWER_HEIGHT_INCHES = 72.125;

    public static final double TOWER_WIDTH_METERS = Units.inchesToMeters(TOWER_WIDTH_INCHES);
    public static final double TOWER_DEPTH_METERS = Units.inchesToMeters(TOWER_DEPTH_INCHES);
    public static final double TOWER_HEIGHT_METERS = Units.inchesToMeters(TOWER_HEIGHT_INCHES);

    // Rung heights (controlled dimensions, GE-26500 sheet 2)
    public static final double TOWER_LOW_RUNG_HEIGHT_INCHES = 27.0;
    public static final double TOWER_MID_RUNG_HEIGHT_INCHES = 45.0;
    public static final double TOWER_HIGH_RUNG_HEIGHT_INCHES = 63.0;
    public static final double TOWER_LOW_RUNG_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_LOW_RUNG_HEIGHT_INCHES);
    public static final double TOWER_MID_RUNG_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_MID_RUNG_HEIGHT_INCHES);
    public static final double TOWER_HIGH_RUNG_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_HIGH_RUNG_HEIGHT_INCHES);

    // Rung extension from upright (GE-26500 sheet 2: "3x 5.875")
    public static final double TOWER_RUNG_HALF_EXTENSION_INCHES = 5.875;
    public static final double TOWER_RUNG_HALF_EXTENSION_METERS =
        Units.inchesToMeters(TOWER_RUNG_HALF_EXTENSION_INCHES);

    // Blue TOWER position
    public static final double BLUE_TOWER_X_INCHES = 0.32; // at Blue wall (from AT_31/32)
    public static final double BLUE_TOWER_CENTER_Y_INCHES =
        161.72; // avg of AT_31 and AT_32 WPILib Y
    public static final double BLUE_TOWER_X_METERS = Units.inchesToMeters(BLUE_TOWER_X_INCHES);
    public static final double BLUE_TOWER_CENTER_Y_METERS =
        Units.inchesToMeters(BLUE_TOWER_CENTER_Y_INCHES);

    // Red TOWER position
    public static final double RED_TOWER_X_INCHES = 650.90; // at Red wall (from AT_15/16)
    public static final double RED_TOWER_CENTER_Y_INCHES =
        155.97; // avg of AT_15 and AT_16 WPILib Y
    public static final double RED_TOWER_X_METERS = Units.inchesToMeters(RED_TOWER_X_INCHES);
    public static final double RED_TOWER_CENTER_Y_METERS =
        Units.inchesToMeters(RED_TOWER_CENTER_Y_INCHES);

    // ===== OUTPOST POSITIONS =====
    // Source: FE-2026 Sheet 11 AprilTag table (Welded Perimeter)
    //
    // Blue OUTPOST (at Blue alliance wall, scoring-table side corner):
    //   AT_29: X=0.30, Y=26.22  → WPILib (0.30, 291.47)
    //   AT_30: X=0.30, Y=43.22  → WPILib (0.30, 274.47)
    //   Center WPILib Y = (291.47 + 274.47) / 2 = 282.97"
    //
    // Red OUTPOST (at Red alliance wall, audience side corner):
    //   AT_13: X=650.92, Y=291.47 → WPILib (650.92, 26.22)
    //   AT_14: X=650.92, Y=274.47 → WPILib (650.92, 43.22)
    //   Center WPILib Y = (26.22 + 43.22) / 2 = 34.72"
    //
    // NOTE: Blue Outpost is near the SCORING TABLE side; Red Outpost is near
    // the AUDIENCE side — they are on OPPOSITE walls. The old code had both
    // at the same relative Y (FIELD_WIDTH - CORRAL_DEPTH), which was wrong
    // for at least one alliance.
    //
    // OUTPOST dimensions from GE-26000 sheet 2 of 2:
    //   Overall frame width: 32.00"
    //   Overall height: 28.13 ±0.25" (to chute opening bottom)
    //   Chute opening: ~17" × 7"
    //   Corral: 15.17" × 17.00" footprint
    //   AprilTag center heights: 21.75" (Z) ✓

    // Outpost footprint (GE-26000 drawing dimensions)
    public static final double OUTPOST_WIDTH_INCHES = 32.00; // along Y (face width)
    public static final double OUTPOST_DEPTH_INCHES = 17.00; // along X (into field)
    public static final double OUTPOST_HEIGHT_INCHES = 28.13; // to chute bottom
    public static final double OUTPOST_WIDTH_METERS = Units.inchesToMeters(OUTPOST_WIDTH_INCHES);
    public static final double OUTPOST_DEPTH_METERS = Units.inchesToMeters(OUTPOST_DEPTH_INCHES);
    public static final double OUTPOST_HEIGHT_METERS = Units.inchesToMeters(OUTPOST_HEIGHT_INCHES);

    // Chute opening dimensions (GE-26000)
    public static final double OUTPOST_CHUTE_WIDTH_INCHES = 17.0;
    public static final double OUTPOST_CHUTE_HEIGHT_INCHES = 7.0;
    public static final double OUTPOST_CHUTE_BOTTOM_INCHES = 28.13;
    public static final double OUTPOST_CHUTE_WIDTH_METERS =
        Units.inchesToMeters(OUTPOST_CHUTE_WIDTH_INCHES);
    public static final double OUTPOST_CHUTE_HEIGHT_METERS =
        Units.inchesToMeters(OUTPOST_CHUTE_HEIGHT_INCHES);
    public static final double OUTPOST_CHUTE_BOTTOM_METERS =
        Units.inchesToMeters(OUTPOST_CHUTE_BOTTOM_INCHES);

    // Blue OUTPOST — near scoring-table wall (high WPILib Y)
    public static final double BLUE_OUTPOST_X_INCHES = 0.30;
    public static final double BLUE_OUTPOST_CENTER_Y_INCHES = 282.97;
    public static final double BLUE_OUTPOST_X_METERS = Units.inchesToMeters(BLUE_OUTPOST_X_INCHES);
    public static final double BLUE_OUTPOST_CENTER_Y_METERS =
        Units.inchesToMeters(BLUE_OUTPOST_CENTER_Y_INCHES);

    // Red OUTPOST — near audience wall (low WPILib Y)
    public static final double RED_OUTPOST_X_INCHES = 650.92;
    public static final double RED_OUTPOST_CENTER_Y_INCHES = 34.72;
    public static final double RED_OUTPOST_X_METERS = Units.inchesToMeters(RED_OUTPOST_X_INCHES);
    public static final double RED_OUTPOST_CENTER_Y_METERS =
        Units.inchesToMeters(RED_OUTPOST_CENTER_Y_INCHES);

    // ===== DEPOT POSITIONS =====
    // Source: GE-26600 (Depot drawing) for dimensions.
    // POSITION: No AprilTags on depots — position needs CAD/field-manual verification.
    // Current values are reasonable estimates from field layout; flag for update.
    //
    // DEPOT dimensions from GE-26600:
    //   Outer arm: 42" long × 1.125" tall
    //   Perpendicular arm: 27" long
    //   L-shaped weldment, sits on carpet

    public static final double DEPOT_ARM_LONG_INCHES = 42.0; // long arm
    public static final double DEPOT_ARM_SHORT_INCHES = 27.0; // short arm
    public static final double DEPOT_HEIGHT_INCHES = 1.125;
    public static final double DEPOT_ARM_LONG_METERS = Units.inchesToMeters(DEPOT_ARM_LONG_INCHES);
    public static final double DEPOT_ARM_SHORT_METERS =
        Units.inchesToMeters(DEPOT_ARM_SHORT_INCHES);
    public static final double DEPOT_HEIGHT_METERS = Units.inchesToMeters(DEPOT_HEIGHT_INCHES);

    // TODO: Verify depot Y positions against field manual or CAD.
    // Blue DEPOT is in the Blue alliance zone, near the audience-side (opposite to Blue OUTPOST).
    // Red DEPOT is in the Red alliance zone, near the scoring-table-side (opposite to Red OUTPOST).
    // Placeholder X=24" (from alliance wall) is a reasonable estimate from GE-26600 layout.
    public static final double BLUE_DEPOT_X_INCHES = 24.0; // ~24" from Blue wall (TODO: verify)
    public static final double BLUE_DEPOT_Y_INCHES = 35.0; // near audience side (TODO: verify)
    public static final double BLUE_DEPOT_X_METERS = Units.inchesToMeters(BLUE_DEPOT_X_INCHES);
    public static final double BLUE_DEPOT_Y_METERS = Units.inchesToMeters(BLUE_DEPOT_Y_INCHES);

    public static final double RED_DEPOT_X_INCHES = FIELD_LENGTH_INCHES - 24.0; // TODO: verify
    public static final double RED_DEPOT_Y_INCHES =
        FIELD_WIDTH_INCHES - 35.0; // near scoring table (TODO: verify)
    public static final double RED_DEPOT_X_METERS = Units.inchesToMeters(RED_DEPOT_X_INCHES);
    public static final double RED_DEPOT_Y_METERS = Units.inchesToMeters(RED_DEPOT_Y_INCHES);

    // ===== APRILTAG HEIGHTS =====
    // Source: FE-2026 Sheet 11 (Z column of AprilTag table)
    public static final double APRILTAG_HUB_Z_INCHES = 44.25;
    public static final double APRILTAG_TRENCH_Z_INCHES = 35.00;
    public static final double APRILTAG_OUTPOST_Z_INCHES = 21.75;
    public static final double APRILTAG_TOWER_Z_INCHES = 21.75;
    public static final double APRILTAG_HUB_Z_METERS = Units.inchesToMeters(APRILTAG_HUB_Z_INCHES);
    public static final double APRILTAG_TRENCH_Z_METERS =
        Units.inchesToMeters(APRILTAG_TRENCH_Z_INCHES);
    public static final double APRILTAG_OUTPOST_Z_METERS =
        Units.inchesToMeters(APRILTAG_OUTPOST_Z_INCHES);
    public static final double APRILTAG_TOWER_Z_METERS =
        Units.inchesToMeters(APRILTAG_TOWER_Z_INCHES);

    // ===== SCORING ELEMENT (FUEL) =====
    // Source: GE-26900 drawing
    public static final double FUEL_DIAMETER_INCHES = 5.906; // nominal (±0.197)
    public static final double FUEL_DIAMETER_METERS = Units.inchesToMeters(FUEL_DIAMETER_INCHES);
    public static final double FUEL_MIN_WEIGHT_LBS = 0.448;
    public static final double FUEL_MAX_WEIGHT_LBS = 0.500;

    public static final int FUEL_PER_DEPOT = 24;
    public static final int FUEL_PER_OUTPOST_CHUTE = 24;
    public static final int FUEL_MAX_PRELOAD_PER_ROBOT = 8;
    public static final int FUEL_TOTAL = 504;

    // ===== MATCH TIMING =====
    public static final double AUTO_DURATION_SECONDS = 20.0;
    public static final double TRANSITION_SHIFT_DURATION_SECONDS = 10.0;
    public static final double ALLIANCE_SHIFT_DURATION_SECONDS = 25.0;
    public static final double END_GAME_DURATION_SECONDS = 30.0;
    public static final double TELEOP_DURATION_SECONDS = 140.0;
    public static final double MATCH_DURATION_SECONDS = 160.0;

    public static final double SHIFT_1_START_TIME = 130.0;
    public static final double SHIFT_2_START_TIME = 105.0;
    public static final double SHIFT_3_START_TIME = 80.0;
    public static final double SHIFT_4_START_TIME = 55.0;
    public static final double END_GAME_START_TIME = 30.0;

    // ===== POINT VALUES =====
    public static final int FUEL_SCORE_ACTIVE_HUB = 1;
    public static final int FUEL_SCORE_INACTIVE_HUB = 0;

    public static final int TOWER_LEVEL_1_AUTO_POINTS = 15;
    public static final int TOWER_LEVEL_1_TELEOP_POINTS = 10;
    public static final int TOWER_LEVEL_2_POINTS = 20;
    public static final int TOWER_LEVEL_3_POINTS = 30;

    public static final int ENERGIZED_RP_THRESHOLD = 100;
    public static final int SUPERCHARGED_RP_THRESHOLD = 360;
    public static final int TRAVERSAL_RP_THRESHOLD = 50;

    // ===== APRILTAG POSES =====
    // Source: FE-2026 Sheet 11, AndyMark Perimeter table.
    // (FIRST North Carolina uses AndyMark perimeter — FE-2026 Sheet 2 BOM table.)
    //
    // All positions are direct from the Sheet 11 AndyMark column, converted to WPILib:
    //   WPILib_X = AprilTag_X  (same origin and direction)
    //   WPILib_Y = FIELD_WIDTH_INCHES - AprilTag_Y  (Y axis is flipped)
    //   Z = as listed (carpet = 0, up = positive)
    //
    // To switch to Welded perimeter, replace every numeric literal here with the
    // corresponding value from the Welded column on Sheet 11.
    public static Translation3d getAprilTagPose(int id) {
      return switch (id) {
          // ---- TRENCH, Red ---- (AndyMark)
        case 1 -> at(467.08, 291.79, 35.00); // WPILib Y = 316.64-291.79=24.85
        case 6 -> at(467.08, 24.85, 35.00); // WPILib Y = 316.64-24.85=291.79
        case 7 -> at(470.03, 24.85, 35.00);
        case 12 -> at(470.03, 291.79, 35.00);

          // ---- HUB, Red ---- (AndyMark)
        case 2 -> at(468.56, 182.08, 44.25); // Red Hub, 90°  (audience-facing)
        case 3 -> at(444.80, 172.32, 44.25); // Red Hub, 180° (Blue-facing)
        case 4 -> at(444.80, 158.32, 44.25); // Red Hub, 180°
        case 5 -> at(468.56, 134.56, 44.25); // Red Hub, 270° (scoring-table-facing)
        case 8 -> at(482.56, 134.56, 44.25); // Red Hub, 270°
        case 9 -> at(492.33, 144.32, 44.25); // Red Hub, 0°   (Red-facing)
        case 10 -> at(492.33, 158.32, 44.25); // Red Hub, 0°
        case 11 -> at(482.56, 182.08, 44.25); // Red Hub, 90°

          // ---- OUTPOST, Red ---- (AndyMark)
        case 13 -> at(649.58, 291.02, 21.75); // WPILib Y = 316.64-291.02=25.62
        case 14 -> at(649.58, 274.02, 21.75); // WPILib Y = 316.64-274.02=42.62

          // ---- TOWER, Red ---- (AndyMark)
        case 15 -> at(649.57, 169.78, 21.75); // WPILib Y = 316.64-169.78=146.86
        case 16 -> at(649.57, 152.78, 21.75); // WPILib Y = 316.64-152.78=163.86

          // ---- TRENCH, Blue ---- (AndyMark)
        case 17 -> at(183.03, 24.85, 35.00); // WPILib Y = 316.64-24.85=291.79
        case 22 -> at(183.03, 291.79, 35.00); // WPILib Y = 316.64-291.79=24.85
        case 23 -> at(180.08, 291.79, 35.00);
        case 28 -> at(180.08, 24.85, 35.00);

          // ---- HUB, Blue ---- (AndyMark)
        case 18 -> at(181.56, 134.56, 44.25); // Blue Hub, 270° (scoring-table-facing)
        case 19 -> at(205.32, 144.32, 44.25); // Blue Hub, 0°   (Red-facing)
        case 20 -> at(205.32, 158.32, 44.25); // Blue Hub, 0°
        case 21 -> at(181.56, 182.08, 44.25); // Blue Hub, 90°  (audience-facing)
        case 24 -> at(167.56, 182.08, 44.25); // Blue Hub, 90°
        case 25 -> at(157.79, 172.32, 44.25); // Blue Hub, 180° (Blue-facing)
        case 26 -> at(157.79, 158.32, 44.25); // Blue Hub, 180°
        case 27 -> at(167.56, 134.56, 44.25); // Blue Hub, 270°

          // ---- OUTPOST, Blue ---- (AndyMark)
        case 29 -> at(0.54, 25.62, 21.75); // WPILib Y = 316.64-25.62=291.02
        case 30 -> at(0.54, 42.62, 21.75); // WPILib Y = 316.64-42.62=274.02

          // ---- TOWER, Blue ---- (AndyMark)
        case 31 -> at(0.55, 146.86, 21.75); // WPILib Y = 316.64-146.86=169.78
        case 32 -> at(0.55, 163.86, 21.75); // WPILib Y = 316.64-163.86=152.78

        default -> new Translation3d(0, 0, 0);
      };
    }

    /**
     * Converts an AprilTag table entry (X, Y_apriltag, Z) to a WPILib Translation3d. AprilTag Y is
     * measured from the Scoring Table wall; WPILib Y from the Audience wall.
     *
     * <p>The Y-flip reference is the AndyMark interior field width (316.64") as used in the Sheet
     * 11 AndyMark table. The carpet field width (317.69") is preserved in FIELD_WIDTH_INCHES for
     * robot pose estimation but the AprilTag origin for the AndyMark perimeter is the extrusion
     * frame face, not the carpet edge.
     */
    private static final double APRILTAG_Y_ORIGIN_INCHES = 316.64; // AndyMark perimeter ref

    private static Translation3d at(double apriltagX, double apriltagY, double z) {
      return new Translation3d(
          Units.inchesToMeters(apriltagX),
          Units.inchesToMeters(APRILTAG_Y_ORIGIN_INCHES - apriltagY),
          Units.inchesToMeters(z));
    }
  }

  public static final class TrajectoryConstants {
    public static final double MAX_SPEED = 5.0;
    public static final double MAX_ACCELERATION = 3;

    public static final double AUTO_TRANSLATION_P = 1.5;
    public static final double AUTO_TRANSLATION_D = 0.2;
    public static final double AUTO_THETA_P = 4.5;
    public static final double AUTO_THETA_D = 0.4;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2;

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final double X_TOLERANCE = 0.02;
    public static final double Y_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = 1.25;

    public static final double AUTO_ALIGN_TRANSLATIONAL_P = 3;
    public static final double AUTO_ALIGN_TRANSLATIONAL_I = 0;
    public static final double AUTO_ALIGN_TRANSLATIONAL_D = 0;

    public static final double AUTO_ALIGN_ROTATIONAL_P = 3;
    public static final double AUTO_ALIGN_ROTATIONAL_I = 0;
    public static final double AUTO_ALIGN_ROTATIONAL_D = 0;
  }

  public static final class AutoConstants {
    public static final String Y_ONE_METER_AUTO = "Y-One-Meter-Test";
    public static final String Y_ONE_METER_TRAJECTORY = "MiscTrajectories/one_point_one_meter";
  }

  public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;
    public static final double DEADBAND_VALUE = 0.05;
  }
}
