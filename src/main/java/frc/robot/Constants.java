// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
   private static RobotType robotType = RobotType.COMP_ROBOT;

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
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.2);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);

    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2);
    //In addition to this, coordinates for all relevant game structures must be added (ex: blue outpost)
  }
   public static final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final double DEADBAND_VALUE = 0.05;
  }
}
