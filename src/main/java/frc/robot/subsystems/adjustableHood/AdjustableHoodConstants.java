package frc.robot.subsystems.adjustableHood;

public class AdjustableHoodConstants {
  public static final int HOOD_MOTOR_ID = 30;
  public static final int CANCODER_ID = 0;

  public static final double HOOD_MAX_ANGLE = 0.5;
  public static final double GEAR_RATIO = 13.535;

  // Tune G first, increase until hood moves then tune pid and sva
  public static final double HOOD_P = 0.5;
  public static final double HOOD_I = 0;
  public static final double HOOD_D = 0.1;
  public static final double HOOD_S = 0.4;
  public static final double HOOD_V = 0;
  public static final double HOOD_A = 0;
  public static final double HOOD_G = 0;

  public static final double HOOD_ZERO_ANGLE = -0.4817;

  // Distance from hub {Distance (meters), angle}
  public static double[][] hoodLookUpTable = {
    {5, 0.1},
    {6, 0.15},
    {7, 0.2},
    {8, 0.27},
    {9, 0.35},
    {10, 0.4},
    {11, 0.5},
    // {4, 0},
    // {4.5, 0},
    // {5, 0.45}
    // {5.5, 0},
    // {6, 0},
    // {6.5, 0},
    // {7, 0},
    // {7.5, 0},
    // {8, 0}
  };
}
