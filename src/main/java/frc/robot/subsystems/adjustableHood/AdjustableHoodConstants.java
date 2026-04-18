package frc.robot.subsystems.adjustableHood;

public class AdjustableHoodConstants {
  public static final int HOOD_MOTOR_ID = 30;
  public static final int CANCODER_ID = 50;

  // public static final double HOOD_MAX_ANGLE = 0.5;
  public static final double GEAR_RATIO = 24 / 42;

  // Tune G first, increase until hood moves then tune pid and sva
  public static final double HOOD_P = 50; // 75
  public static final double HOOD_I = 0.0;
  public static final double HOOD_D = 0.0;
  public static final double HOOD_S = 0.23;
  public static final double HOOD_V = 0.0; // 0.08
  public static final double HOOD_A = 0;
  public static final double HOOD_G = 0;

  public static final double HOOD_ZERO_ANGLE = 0.199707;

  // Distance from hub {Distance (meters), angle}
  public static double[][] hoodLookUpTable = {
    {1.5, .4},
    {2.0, .4829},
    {2.5, 0.53},
    {3.0, .555},
    {3.5, .68},
    {4.0, 0.75},
    {4.5, 0.82},
    {5.0, .85},
    // FOR PASSING ONLY - GUESSED
    {6, .90},
    {10, 1.7},
    {13, 1.7}
  };
}
