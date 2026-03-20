package frc.robot.subsystems.adjustableHood;

public class AdjustableHoodConstants {
  public static final int HOOD_MOTOR_ID = 30;
  public static final int CANCODER_ID = 50;

  public static final double HOOD_MAX_ANGLE = 0.5;
  public static final double GEAR_RATIO = 1;

  // Tune G first, increase until hood moves then tune pid and sva
  public static final double HOOD_P = 50;
  public static final double HOOD_I = 0.0;
  public static final double HOOD_D = 0.00;
  public static final double HOOD_S = 0.4;
  public static final double HOOD_V = 0.08;
  public static final double HOOD_A = 0;
  public static final double HOOD_G = 0;

  // (Best thus far, P = 0.59)
  // (Best thus far, D = ) (1-2% of P)

  public static final double HOOD_ZERO_ANGLE = 0.199707;

  // Distance from hub {Distance (meters), angle}
  public static double[][] hoodLookUpTable = {
    {1, 0.1},
    {2, 0.2},
    {3, 0.3},
    {4, 0.35},
    {5, 0.4},
    {6, 0.45},
    {7, 0.5},
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
