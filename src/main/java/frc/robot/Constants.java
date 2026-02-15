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

  /**
   * Gets if the robot type is valid, if not it will default to COMP_ROBOT
   *
   * @return the currently used RobotType
   */
  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    // if (RobotBase.isReal() && robotType == RobotType.SIM_ROBOT) {
    //   new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
    //       .set(true);
    //   robotType = RobotType.COMP_ROBOT;
    // }
    return robotType;
  }

  /**
   * Gets the mode of the robot based on the RobotType and the state of {@link RobotBase}, if the
   * robot isn't real but is also not the SIM_ROBOT, it will set the currently used mode to REPLAY
   *
   * @return the currently used Mode
   */
  public static Mode getMode() {
    return switch (robotType) {
      case DEV_ROBOT, COMP_ROBOT, SWERVE_ROBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM_ROBOT -> Mode.SIM;
    };
  }

  /** An enum to select the robot's mode. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** An enum to select the currently used robot. */
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

    /**
     * For some reason, falcons normally have a deadband threshold of 4%. This is incredibly high!
     * It makes it very hard to do precise movements, so with this constant we set the threshold to
     * the lowest possible value.
     */
    public static final double MIN_DUTY_CYCLE_DEADBAND = 0.001;

    public static final double MIN_TORQUE_DEADBAND = 5.0;

    public static final int HIGH_THREAD_PRIORITY = 99;
    public static final int LOW_THREAD_PRIORITY = 1;
  }

  public static final class FieldConstants {
    // Field dimensions (from manual section 5.2)
    public static final double FIELD_LENGTH_INCHES = 651.2;
    public static final double FIELD_WIDTH_INCHES = 317.7;

    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(FIELD_LENGTH_INCHES);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(FIELD_WIDTH_INCHES);

    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2);

    public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(72);
    public static final double HUB_LENGTH_METERS = Units.inchesToMeters(47);
    public static final double HUB_LIP_LENGTH_METERS = Units.inchesToMeters(41.7);

    public static final double HUB_WALL_FROM_ALLIANCE_WALL_METERS = Units.inchesToMeters(158.6);

    public static final Translation2d RED_HUB_CENTER =
        new Translation2d(
            HUB_WALL_FROM_ALLIANCE_WALL_METERS + (HUB_LENGTH_METERS / 2), FIELD_WIDTH_METERS / 2);
    public static final Translation2d BLUE_HUB_CENTER =
        new Translation2d(
            FIELD_LENGTH_METERS - (HUB_WALL_FROM_ALLIANCE_WALL_METERS + (HUB_LENGTH_METERS / 2)),
            FIELD_WIDTH_METERS / 2);

    // Center line and zones (from manual section 5.3)
    public static final double CENTER_LINE_X_INCHES = FIELD_LENGTH_INCHES / 2.0;
    public static final double CENTER_LINE_X_METERS = Units.inchesToMeters(CENTER_LINE_X_INCHES);

    // Alliance zone depth (158.6" from alliance wall to robot starting line)
    public static final double ALLIANCE_ZONE_DEPTH_INCHES = 158.6;
    public static final double ALLIANCE_ZONE_DEPTH_METERS =
        Units.inchesToMeters(ALLIANCE_ZONE_DEPTH_INCHES);

    // Neutral zone depth (283" from robot starting line to robot starting line)
    public static final double NEUTRAL_ZONE_DEPTH_INCHES = 283.0;
    public static final double NEUTRAL_ZONE_DEPTH_METERS =
        Units.inchesToMeters(NEUTRAL_ZONE_DEPTH_INCHES);

    // Human starting line (24" from alliance wall)
    public static final double HUMAN_STARTING_LINE_OFFSET_INCHES = 24.0;
    public static final double HUMAN_STARTING_LINE_OFFSET_METERS =
        Units.inchesToMeters(HUMAN_STARTING_LINE_OFFSET_INCHES);

    // Outpost area dimensions
    public static final double OUTPOST_AREA_WIDTH_INCHES = 71.0;
    public static final double OUTPOST_AREA_DEPTH_INCHES = 134.0;
    public static final double OUTPOST_AREA_WIDTH_METERS =
        Units.inchesToMeters(OUTPOST_AREA_WIDTH_INCHES);
    public static final double OUTPOST_AREA_DEPTH_METERS =
        Units.inchesToMeters(OUTPOST_AREA_DEPTH_INCHES);

    // ===== HUB positions (from manual section 5.4 and field drawings) =====
    // HUBs are centered between two BUMPS, 158.6" from alliance wall
    public static final double HUB_DISTANCE_FROM_WALL_INCHES = 158.6;
    public static final double HUB_DISTANCE_FROM_WALL_METERS =
        Units.inchesToMeters(HUB_DISTANCE_FROM_WALL_INCHES);

    // Blue HUB position (from blue alliance wall)
    public static final double BLUE_HUB_X_INCHES = HUB_DISTANCE_FROM_WALL_INCHES;
    public static final double BLUE_HUB_Y_INCHES = FIELD_WIDTH_INCHES / 2.0; // Centered width-wise
    public static final double BLUE_HUB_Z_INCHES = 72.0; // Top opening height

    public static final double BLUE_HUB_X_METERS = Units.inchesToMeters(BLUE_HUB_X_INCHES);
    public static final double BLUE_HUB_Y_METERS = Units.inchesToMeters(BLUE_HUB_Y_INCHES);
    public static final double BLUE_HUB_Z_METERS = Units.inchesToMeters(BLUE_HUB_Z_INCHES);

    // Red HUB position (from red alliance wall)
    public static final double RED_HUB_X_INCHES =
        FIELD_LENGTH_INCHES - HUB_DISTANCE_FROM_WALL_INCHES;
    public static final double RED_HUB_Y_INCHES = FIELD_WIDTH_INCHES / 2.0; // Centered width-wise
    public static final double RED_HUB_Z_INCHES = 72.0; // Top opening height

    public static final double RED_HUB_X_METERS = Units.inchesToMeters(RED_HUB_X_INCHES);
    public static final double RED_HUB_Y_METERS = Units.inchesToMeters(RED_HUB_Y_INCHES);
    public static final double RED_HUB_Z_METERS = Units.inchesToMeters(RED_HUB_Z_INCHES);

    // HUB dimensions (47" x 47" square)
    public static final double HUB_WIDTH_INCHES = 47.0;
    public static final double HUB_WIDTH_METERS = Units.inchesToMeters(HUB_WIDTH_INCHES);
    public static final double HUB_DEPTH_INCHES = 47.0;
    public static final double HUB_DEPTH_METERS = Units.inchesToMeters(HUB_DEPTH_INCHES);

    // HUB opening dimensions (41.7" hex opening)
    public static final double HUB_OPENING_SIZE_INCHES = 41.7;
    public static final double HUB_OPENING_SIZE_METERS =
        Units.inchesToMeters(HUB_OPENING_SIZE_INCHES);

    // ===== TOWER positions (from manual section 5.8) =====
    // TOWER integrated into alliance wall between Driver Stations 2 & 3
    public static final double TOWER_WIDTH_INCHES = 49.25;
    public static final double TOWER_DEPTH_INCHES = 45.0;
    public static final double TOWER_HEIGHT_INCHES = 78.25;

    public static final double TOWER_WIDTH_METERS = Units.inchesToMeters(TOWER_WIDTH_INCHES);
    public static final double TOWER_DEPTH_METERS = Units.inchesToMeters(TOWER_DEPTH_INCHES);
    public static final double TOWER_HEIGHT_METERS = Units.inchesToMeters(TOWER_HEIGHT_INCHES);

    // TOWER rung heights (from floor)
    public static final double TOWER_LOW_RUNG_HEIGHT_INCHES = 27.0;
    public static final double TOWER_MID_RUNG_HEIGHT_INCHES = 45.0;
    public static final double TOWER_HIGH_RUNG_HEIGHT_INCHES = 63.0;

    public static final double TOWER_LOW_RUNG_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_LOW_RUNG_HEIGHT_INCHES);
    public static final double TOWER_MID_RUNG_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_MID_RUNG_HEIGHT_INCHES);
    public static final double TOWER_HIGH_RUNG_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_HIGH_RUNG_HEIGHT_INCHES);

    // TOWER rung extension from upright (5.87" on each side)
    public static final double TOWER_RUNG_EXTENSION_INCHES = 5.875;
    public static final double TOWER_RUNG_EXTENSION_METERS =
        Units.inchesToMeters(TOWER_RUNG_EXTENSION_INCHES);

    // Blue TOWER position (along blue alliance wall)
    public static final double BLUE_TOWER_X_INCHES = 0.0; // At alliance wall
    public static final double BLUE_TOWER_Y_INCHES = FIELD_WIDTH_INCHES * 0.4; // Approx position

    public static final double BLUE_TOWER_X_METERS = Units.inchesToMeters(BLUE_TOWER_X_INCHES);
    public static final double BLUE_TOWER_Y_METERS = Units.inchesToMeters(BLUE_TOWER_Y_INCHES);

    // Red TOWER position (along red alliance wall)
    public static final double RED_TOWER_X_INCHES = FIELD_LENGTH_INCHES;
    public static final double RED_TOWER_Y_INCHES = FIELD_WIDTH_INCHES * 0.4;

    public static final double RED_TOWER_X_METERS = Units.inchesToMeters(RED_TOWER_X_INCHES);
    public static final double RED_TOWER_Y_METERS = Units.inchesToMeters(RED_TOWER_Y_INCHES);

    // TOWER base plate dimensions
    public static final double TOWER_BASE_WIDTH_INCHES = 39.0;
    public static final double TOWER_BASE_DEPTH_INCHES = 45.18;
    public static final double TOWER_BASE_HEIGHT_INCHES = 0.2; // Edge height

    public static final double TOWER_BASE_WIDTH_METERS =
        Units.inchesToMeters(TOWER_BASE_WIDTH_INCHES);
    public static final double TOWER_BASE_DEPTH_METERS =
        Units.inchesToMeters(TOWER_BASE_DEPTH_INCHES);
    public static final double TOWER_BASE_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_BASE_HEIGHT_INCHES);

    // ===== BUMP positions (from manual section 5.5) =====
    // BUMP dimensions
    public static final double BUMP_WIDTH_INCHES = 73.0;
    public static final double BUMP_DEPTH_INCHES = 44.4;
    public static final double BUMP_HEIGHT_INCHES = 6.513;
    public static final double BUMP_ANGLE_DEGREES = 15.0;

    public static final double BUMP_WIDTH_METERS = Units.inchesToMeters(BUMP_WIDTH_INCHES);
    public static final double BUMP_DEPTH_METERS = Units.inchesToMeters(BUMP_DEPTH_INCHES);
    public static final double BUMP_HEIGHT_METERS = Units.inchesToMeters(BUMP_HEIGHT_INCHES);

    // Blue alliance BUMPS (two on each side of HUB)
    public static final double BLUE_BUMP_LEFT_X_INCHES = BLUE_HUB_X_INCHES - BUMP_WIDTH_INCHES;
    public static final double BLUE_BUMP_LEFT_Y_INCHES = BLUE_HUB_Y_INCHES - BUMP_DEPTH_INCHES / 2;

    public static final double BLUE_BUMP_RIGHT_X_INCHES = BLUE_HUB_X_INCHES;
    public static final double BLUE_BUMP_RIGHT_Y_INCHES = BLUE_HUB_Y_INCHES - BUMP_DEPTH_INCHES / 2;

    public static final double BLUE_BUMP_LEFT_X_METERS =
        Units.inchesToMeters(BLUE_BUMP_LEFT_X_INCHES);
    public static final double BLUE_BUMP_LEFT_Y_METERS =
        Units.inchesToMeters(BLUE_BUMP_LEFT_Y_INCHES);
    public static final double BLUE_BUMP_RIGHT_X_METERS =
        Units.inchesToMeters(BLUE_BUMP_RIGHT_X_INCHES);
    public static final double BLUE_BUMP_RIGHT_Y_METERS =
        Units.inchesToMeters(BLUE_BUMP_RIGHT_Y_INCHES);

    // Red alliance BUMPS
    public static final double RED_BUMP_LEFT_X_INCHES = RED_HUB_X_INCHES - BUMP_WIDTH_INCHES;
    public static final double RED_BUMP_LEFT_Y_INCHES = RED_HUB_Y_INCHES - BUMP_DEPTH_INCHES / 2;

    public static final double RED_BUMP_RIGHT_X_INCHES = RED_HUB_X_INCHES;
    public static final double RED_BUMP_RIGHT_Y_INCHES = RED_HUB_Y_INCHES - BUMP_DEPTH_INCHES / 2;

    public static final double RED_BUMP_LEFT_X_METERS =
        Units.inchesToMeters(RED_BUMP_LEFT_X_INCHES);
    public static final double RED_BUMP_LEFT_Y_METERS =
        Units.inchesToMeters(RED_BUMP_LEFT_Y_INCHES);
    public static final double RED_BUMP_RIGHT_X_METERS =
        Units.inchesToMeters(RED_BUMP_RIGHT_X_INCHES);
    public static final double RED_BUMP_RIGHT_Y_METERS =
        Units.inchesToMeters(RED_BUMP_RIGHT_Y_INCHES);

    // ===== TRENCH positions (from manual section 5.6) =====
    // TRENCH dimensions
    public static final double TRENCH_WIDTH_INCHES = 65.65;
    public static final double TRENCH_DEPTH_INCHES = 47.0;
    public static final double TRENCH_HEIGHT_INCHES = 40.25;
    public static final double TRENCH_UNDER_WIDTH_INCHES = 50.34;
    public static final double TRENCH_UNDER_HEIGHT_INCHES = 22.25;

    public static final double TRENCH_WIDTH_METERS = Units.inchesToMeters(TRENCH_WIDTH_INCHES);
    public static final double TRENCH_DEPTH_METERS = Units.inchesToMeters(TRENCH_DEPTH_INCHES);
    public static final double TRENCH_HEIGHT_METERS = Units.inchesToMeters(TRENCH_HEIGHT_INCHES);
    public static final double TRENCH_UNDER_WIDTH_METERS =
        Units.inchesToMeters(TRENCH_UNDER_WIDTH_INCHES);
    public static final double TRENCH_UNDER_HEIGHT_METERS =
        Units.inchesToMeters(TRENCH_UNDER_HEIGHT_INCHES);

    // Blue alliance TRENCHES (extend from guardrail to BUMP)
    public static final double BLUE_TRENCH_LEFT_X_INCHES = 0; // Starts at guardrail
    public static final double BLUE_TRENCH_LEFT_Y_INCHES =
        BLUE_HUB_Y_INCHES - TRENCH_DEPTH_INCHES / 2;

    public static final double BLUE_TRENCH_RIGHT_X_INCHES = FIELD_LENGTH_INCHES / 4;
    public static final double BLUE_TRENCH_RIGHT_Y_INCHES =
        BLUE_HUB_Y_INCHES - TRENCH_DEPTH_INCHES / 2;

    public static final double BLUE_TRENCH_LEFT_X_METERS =
        Units.inchesToMeters(BLUE_TRENCH_LEFT_X_INCHES);
    public static final double BLUE_TRENCH_LEFT_Y_METERS =
        Units.inchesToMeters(BLUE_TRENCH_LEFT_Y_INCHES);
    public static final double BLUE_TRENCH_RIGHT_X_METERS =
        Units.inchesToMeters(BLUE_TRENCH_RIGHT_X_INCHES);
    public static final double BLUE_TRENCH_RIGHT_Y_METERS =
        Units.inchesToMeters(BLUE_TRENCH_RIGHT_Y_INCHES);

    // Red alliance TRENCHES
    public static final double RED_TRENCH_LEFT_X_INCHES = FIELD_LENGTH_INCHES * 0.75;
    public static final double RED_TRENCH_LEFT_Y_INCHES =
        RED_HUB_Y_INCHES - TRENCH_DEPTH_INCHES / 2;

    public static final double RED_TRENCH_RIGHT_X_INCHES =
        FIELD_LENGTH_INCHES - TRENCH_WIDTH_INCHES;
    public static final double RED_TRENCH_RIGHT_Y_INCHES =
        RED_HUB_Y_INCHES - TRENCH_DEPTH_INCHES / 2;

    public static final double RED_TRENCH_LEFT_X_METERS =
        Units.inchesToMeters(RED_TRENCH_LEFT_X_INCHES);
    public static final double RED_TRENCH_LEFT_Y_METERS =
        Units.inchesToMeters(RED_TRENCH_LEFT_Y_INCHES);
    public static final double RED_TRENCH_RIGHT_X_METERS =
        Units.inchesToMeters(RED_TRENCH_RIGHT_X_INCHES);
    public static final double RED_TRENCH_RIGHT_Y_METERS =
        Units.inchesToMeters(RED_TRENCH_RIGHT_Y_INCHES);

    // ===== DEPOT positions (from manual section 5.7) =====
    // DEPOT dimensions
    public static final double DEPOT_WIDTH_INCHES = 42.0;
    public static final double DEPOT_DEPTH_INCHES = 27.0;
    public static final double DEPOT_HEIGHT_INCHES = 1.125; // Including hook fastener

    public static final double DEPOT_WIDTH_METERS = Units.inchesToMeters(DEPOT_WIDTH_INCHES);
    public static final double DEPOT_DEPTH_METERS = Units.inchesToMeters(DEPOT_DEPTH_INCHES);
    public static final double DEPOT_HEIGHT_METERS = Units.inchesToMeters(DEPOT_HEIGHT_INCHES);

    // Blue DEPOT (along alliance wall)
    public static final double BLUE_DEPOT_X_INCHES = 24.0; // Offset from wall
    public static final double BLUE_DEPOT_Y_INCHES = FIELD_WIDTH_INCHES * 0.15;

    public static final double BLUE_DEPOT_X_METERS = Units.inchesToMeters(BLUE_DEPOT_X_INCHES);
    public static final double BLUE_DEPOT_Y_METERS = Units.inchesToMeters(BLUE_DEPOT_Y_INCHES);

    // Red DEPOT
    public static final double RED_DEPOT_X_INCHES = FIELD_LENGTH_INCHES - 24.0 - DEPOT_WIDTH_INCHES;
    public static final double RED_DEPOT_Y_INCHES = FIELD_WIDTH_INCHES * 0.15;

    public static final double RED_DEPOT_X_METERS = Units.inchesToMeters(RED_DEPOT_X_INCHES);
    public static final double RED_DEPOT_Y_METERS = Units.inchesToMeters(RED_DEPOT_Y_INCHES);

    // ===== OUTPOST positions (from manual section 5.9.2) =====
    // OUTPOST dimensions
    public static final double OUTPOST_CHUTE_OPENING_WIDTH_INCHES = 31.8;
    public static final double OUTPOST_CHUTE_OPENING_HEIGHT_INCHES = 7.0;
    public static final double OUTPOST_CHUTE_OPENING_BOTTOM_INCHES = 28.1;

    public static final double OUTPOST_CORRAL_OPENING_WIDTH_INCHES = 32.0;
    public static final double OUTPOST_CORRAL_OPENING_HEIGHT_INCHES = 7.0;
    public static final double OUTPOST_CORRAL_OPENING_BOTTOM_INCHES = 1.88;

    public static final double OUTPOST_CORRAL_WIDTH_INCHES = 35.8;
    public static final double OUTPOST_CORRAL_DEPTH_INCHES = 37.6;
    public static final double OUTPOST_CORRAL_HEIGHT_INCHES = 8.13;

    public static final double OUTPOST_CHUTE_TAPE_OFFSET_INCHES = 12.9;
    public static final double OUTPOST_CORRAL_TAPE_OFFSET_INCHES = 12.7;

    public static final double OUTPOST_CHUTE_OPENING_WIDTH_METERS =
        Units.inchesToMeters(OUTPOST_CHUTE_OPENING_WIDTH_INCHES);
    public static final double OUTPOST_CHUTE_OPENING_HEIGHT_METERS =
        Units.inchesToMeters(OUTPOST_CHUTE_OPENING_HEIGHT_INCHES);
    public static final double OUTPOST_CHUTE_OPENING_BOTTOM_METERS =
        Units.inchesToMeters(OUTPOST_CHUTE_OPENING_BOTTOM_INCHES);

    public static final double OUTPOST_CORRAL_WIDTH_METERS =
        Units.inchesToMeters(OUTPOST_CORRAL_WIDTH_INCHES);
    public static final double OUTPOST_CORRAL_DEPTH_METERS =
        Units.inchesToMeters(OUTPOST_CORRAL_DEPTH_INCHES);
    public static final double OUTPOST_CORRAL_HEIGHT_METERS =
        Units.inchesToMeters(OUTPOST_CORRAL_HEIGHT_INCHES);

    // Blue OUTPOST (at end of alliance wall)
    public static final double BLUE_OUTPOST_X_INCHES = 0; // At corner of field
    public static final double BLUE_OUTPOST_Y_INCHES =
        FIELD_WIDTH_INCHES - OUTPOST_CORRAL_DEPTH_INCHES;

    public static final double BLUE_OUTPOST_X_METERS = Units.inchesToMeters(BLUE_OUTPOST_X_INCHES);
    public static final double BLUE_OUTPOST_Y_METERS = Units.inchesToMeters(BLUE_OUTPOST_Y_INCHES);

    // Red OUTPOST
    public static final double RED_OUTPOST_X_INCHES =
        FIELD_LENGTH_INCHES - OUTPOST_CORRAL_WIDTH_INCHES;
    public static final double RED_OUTPOST_Y_INCHES =
        FIELD_WIDTH_INCHES - OUTPOST_CORRAL_DEPTH_INCHES;

    public static final double RED_OUTPOST_X_METERS = Units.inchesToMeters(RED_OUTPOST_X_INCHES);
    public static final double RED_OUTPOST_Y_METERS = Units.inchesToMeters(RED_OUTPOST_Y_INCHES);

    // ===== AprilTag positions (from manual section 3.16 and 5.11) =====
    // AprilTag dimensions
    public static final double APRILTAG_SIZE_INCHES = 8.125;
    public static final double APRILTAG_SIZE_METERS = Units.inchesToMeters(APRILTAG_SIZE_INCHES);

    // HUB AprilTags (IDs 2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27)
    public static final double HUB_APRILTAG_HEIGHT_INCHES = 44.25;
    public static final double HUB_APRILTAG_HEIGHT_METERS =
        Units.inchesToMeters(HUB_APRILTAG_HEIGHT_INCHES);

    // TOWER WALL AprilTags (IDs 15,16,31,32)
    public static final double TOWER_WALL_APRILTAG_HEIGHT_INCHES = 21.75;
    public static final double TOWER_WALL_APRILTAG_HEIGHT_METERS =
        Units.inchesToMeters(TOWER_WALL_APRILTAG_HEIGHT_INCHES);

    // OUTPOST AprilTags (IDs 13,14,29,30)
    public static final double OUTPOST_APRILTAG_HEIGHT_INCHES = 21.75;
    public static final double OUTPOST_APRILTAG_HEIGHT_METERS =
        Units.inchesToMeters(OUTPOST_APRILTAG_HEIGHT_INCHES);

    // TRENCH AprilTags (IDs 1,6,7,12,17,22,23,28)
    public static final double TRENCH_APRILTAG_HEIGHT_INCHES = 35.0;
    public static final double TRENCH_APRILTAG_HEIGHT_METERS =
        Units.inchesToMeters(TRENCH_APRILTAG_HEIGHT_INCHES);

    // ===== SCORING ELEMENT (FUEL) specifications =====
    public static final double FUEL_DIAMETER_INCHES = 5.91;
    public static final double FUEL_DIAMETER_METERS = Units.inchesToMeters(FUEL_DIAMETER_INCHES);
    public static final double FUEL_MIN_WEIGHT_LBS = 0.448;
    public static final double FUEL_MAX_WEIGHT_LBS = 0.5;

    // FUEL staging quantities (from manual section 6.3.4)
    public static final int FUEL_PER_DEPOT = 24;
    public static final int FUEL_PER_OUTPOST_CHUTE = 24;
    public static final int FUEL_MAX_PRELOAD_PER_ROBOT = 8;
    public static final int FUEL_TOTAL = 504;

    // ===== MATCH timing (from manual section 6.4) =====
    public static final double AUTO_DURATION_SECONDS = 20.0;
    public static final double TRANSITION_SHIFT_DURATION_SECONDS = 10.0;
    public static final double ALLIANCE_SHIFT_DURATION_SECONDS = 25.0;
    public static final double END_GAME_DURATION_SECONDS = 30.0;
    public static final double TELEOP_DURATION_SECONDS = 140.0; // 2:20
    public static final double MATCH_DURATION_SECONDS = 160.0; // 2:40

    // SHIFT transition times (from end of match)
    public static final double SHIFT_1_START_TIME = 130.0; // 2:10 remaining
    public static final double SHIFT_2_START_TIME = 105.0; // 1:45 remaining
    public static final double SHIFT_3_START_TIME = 80.0; // 1:20 remaining
    public static final double SHIFT_4_START_TIME = 55.0; // 0:55 remaining
    public static final double END_GAME_START_TIME = 30.0; // 0:30 remaining

    // ===== Point values (from manual section 6.5.3) =====
    public static final int FUEL_SCORE_ACTIVE_HUB = 1;
    public static final int FUEL_SCORE_INACTIVE_HUB = 0;

    public static final int TOWER_LEVEL_1_AUTO_POINTS = 15;
    public static final int TOWER_LEVEL_1_TELEOP_POINTS = 10;
    public static final int TOWER_LEVEL_2_POINTS = 20;
    public static final int TOWER_LEVEL_3_POINTS = 30;

    // RP thresholds (from manual section 6.5.3)
    public static final int ENERGIZED_RP_THRESHOLD = 100;
    public static final int SUPERCHARGED_RP_THRESHOLD = 360;
    public static final int TRAVERSAL_RP_THRESHOLD = 50;

    /**
     * Get the translation for a specific AprilTag ID
     *
     * @param id AprilTag ID (1-32)
     * @return Translation3d position of the tag
     */
    public static Translation3d getAprilTagPose(int id) {
      switch (id) {
          // TRENCH tags (1,6,7,12,17,22,23,28)
        case 1:
          return new Translation3d(
              BLUE_TRENCH_LEFT_X_METERS, BLUE_TRENCH_LEFT_Y_METERS, TRENCH_APRILTAG_HEIGHT_METERS);
        case 6:
          return new Translation3d(
              BLUE_TRENCH_RIGHT_X_METERS,
              BLUE_TRENCH_RIGHT_Y_METERS,
              TRENCH_APRILTAG_HEIGHT_METERS);
        case 7:
          return new Translation3d(
              BLUE_TRENCH_RIGHT_X_METERS,
              BLUE_TRENCH_RIGHT_Y_METERS + TRENCH_WIDTH_METERS,
              TRENCH_APRILTAG_HEIGHT_METERS);
        case 12:
          return new Translation3d(
              BLUE_TRENCH_LEFT_X_METERS,
              BLUE_TRENCH_LEFT_Y_METERS + TRENCH_WIDTH_METERS,
              TRENCH_APRILTAG_HEIGHT_METERS);

        case 17:
          return new Translation3d(
              RED_TRENCH_LEFT_X_METERS, RED_TRENCH_LEFT_Y_METERS, TRENCH_APRILTAG_HEIGHT_METERS);
        case 22:
          return new Translation3d(
              RED_TRENCH_RIGHT_X_METERS, RED_TRENCH_RIGHT_Y_METERS, TRENCH_APRILTAG_HEIGHT_METERS);
        case 23:
          return new Translation3d(
              RED_TRENCH_RIGHT_X_METERS,
              RED_TRENCH_RIGHT_Y_METERS + TRENCH_WIDTH_METERS,
              TRENCH_APRILTAG_HEIGHT_METERS);
        case 28:
          return new Translation3d(
              RED_TRENCH_LEFT_X_METERS,
              RED_TRENCH_LEFT_Y_METERS + TRENCH_WIDTH_METERS,
              TRENCH_APRILTAG_HEIGHT_METERS);

          // HUB tags (2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27)
        case 2:
          return new Translation3d(
              BLUE_HUB_X_METERS - HUB_WIDTH_METERS / 2,
              BLUE_HUB_Y_METERS - HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 3:
          return new Translation3d(
              BLUE_HUB_X_METERS - HUB_WIDTH_METERS / 2,
              BLUE_HUB_Y_METERS + HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 4:
          return new Translation3d(
              BLUE_HUB_X_METERS + HUB_WIDTH_METERS / 2,
              BLUE_HUB_Y_METERS + HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 5:
          return new Translation3d(
              BLUE_HUB_X_METERS + HUB_WIDTH_METERS / 2,
              BLUE_HUB_Y_METERS - HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);

        case 8:
          return new Translation3d(
              BLUE_HUB_X_METERS - HUB_WIDTH_METERS / 2,
              BLUE_HUB_Y_METERS,
              HUB_APRILTAG_HEIGHT_METERS);
        case 9:
          return new Translation3d(
              BLUE_HUB_X_METERS,
              BLUE_HUB_Y_METERS + HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 10:
          return new Translation3d(
              BLUE_HUB_X_METERS + HUB_WIDTH_METERS / 2,
              BLUE_HUB_Y_METERS,
              HUB_APRILTAG_HEIGHT_METERS);
        case 11:
          return new Translation3d(
              BLUE_HUB_X_METERS,
              BLUE_HUB_Y_METERS - HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);

        case 18:
          return new Translation3d(
              RED_HUB_X_METERS - HUB_WIDTH_METERS / 2,
              RED_HUB_Y_METERS - HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 19:
          return new Translation3d(
              RED_HUB_X_METERS - HUB_WIDTH_METERS / 2,
              RED_HUB_Y_METERS + HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 20:
          return new Translation3d(
              RED_HUB_X_METERS + HUB_WIDTH_METERS / 2,
              RED_HUB_Y_METERS + HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 21:
          return new Translation3d(
              RED_HUB_X_METERS + HUB_WIDTH_METERS / 2,
              RED_HUB_Y_METERS - HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);

        case 24:
          return new Translation3d(
              RED_HUB_X_METERS - HUB_WIDTH_METERS / 2,
              RED_HUB_Y_METERS,
              HUB_APRILTAG_HEIGHT_METERS);
        case 25:
          return new Translation3d(
              RED_HUB_X_METERS,
              RED_HUB_Y_METERS + HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);
        case 26:
          return new Translation3d(
              RED_HUB_X_METERS + HUB_WIDTH_METERS / 2,
              RED_HUB_Y_METERS,
              HUB_APRILTAG_HEIGHT_METERS);
        case 27:
          return new Translation3d(
              RED_HUB_X_METERS,
              RED_HUB_Y_METERS - HUB_DEPTH_METERS / 2,
              HUB_APRILTAG_HEIGHT_METERS);

          // OUTPOST tags (13,14,29,30)
        case 13:
          return new Translation3d(
              BLUE_OUTPOST_X_METERS + OUTPOST_CORRAL_WIDTH_METERS / 2,
              BLUE_OUTPOST_Y_METERS,
              OUTPOST_APRILTAG_HEIGHT_METERS);
        case 14:
          return new Translation3d(
              BLUE_OUTPOST_X_METERS + OUTPOST_CORRAL_WIDTH_METERS / 2,
              BLUE_OUTPOST_Y_METERS + OUTPOST_CORRAL_DEPTH_METERS / 2,
              OUTPOST_APRILTAG_HEIGHT_METERS);
        case 29:
          return new Translation3d(
              RED_OUTPOST_X_METERS + OUTPOST_CORRAL_WIDTH_METERS / 2,
              RED_OUTPOST_Y_METERS,
              OUTPOST_APRILTAG_HEIGHT_METERS);
        case 30:
          return new Translation3d(
              RED_OUTPOST_X_METERS + OUTPOST_CORRAL_WIDTH_METERS / 2,
              RED_OUTPOST_Y_METERS + OUTPOST_CORRAL_DEPTH_METERS / 2,
              OUTPOST_APRILTAG_HEIGHT_METERS);

          // TOWER tags (15,16,31,32)
        case 15:
          return new Translation3d(
              BLUE_TOWER_X_METERS + TOWER_BASE_DEPTH_METERS / 2,
              BLUE_TOWER_Y_METERS - TOWER_WIDTH_METERS / 2,
              TOWER_WALL_APRILTAG_HEIGHT_METERS);
        case 16:
          return new Translation3d(
              BLUE_TOWER_X_METERS + TOWER_BASE_DEPTH_METERS / 2,
              BLUE_TOWER_Y_METERS + TOWER_WIDTH_METERS / 2,
              TOWER_WALL_APRILTAG_HEIGHT_METERS);
        case 31:
          return new Translation3d(
              RED_TOWER_X_METERS - TOWER_BASE_DEPTH_METERS / 2,
              RED_TOWER_Y_METERS - TOWER_WIDTH_METERS / 2,
              TOWER_WALL_APRILTAG_HEIGHT_METERS);
        case 32:
          return new Translation3d(
              RED_TOWER_X_METERS - TOWER_BASE_DEPTH_METERS / 2,
              RED_TOWER_Y_METERS + TOWER_WIDTH_METERS / 2,
              TOWER_WALL_APRILTAG_HEIGHT_METERS);

        default:
          return new Translation3d(0, 0, 0);
      }
    }
  }

  public static final class TrajectoryConstants {
    public static final double MAX_SPEED = 5.0;
    public static final double MAX_ACCELERATION = 3;

    public static final double AUTO_TRANSLATION_P = 1.5; // 1.7
    public static final double AUTO_TRANSLATION_D = 0.2;
    public static final double AUTO_THETA_P = 4.5; // 5
    public static final double AUTO_THETA_D = 0.4;

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2;

    // Constraint for the motion profiled robot angle controller
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
    // Different Pre-defined Auto Routines
    public static final String Y_ONE_METER_AUTO = "Y-One-Meter-Test";

    public static final String Y_ONE_METER_TRAJECTORY = "MiscTrajectories/one_point_one_meter";
  }

  public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final double DEADBAND_VALUE = 0.05;
  }
}
