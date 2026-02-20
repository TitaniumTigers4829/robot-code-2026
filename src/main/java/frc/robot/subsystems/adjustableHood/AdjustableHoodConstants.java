package frc.robot.subsystems.adjustableHood;

public class AdjustableHoodConstants {
  public static final int HOOD_MOTOR_ID = 30;
  public static final int CANCODER_ID = 14;

  public static final double HOOD_MAX_ANGLE = 90.0;

  // Tune G first, increase until hood moves then tune pid and sva
  public static final double HOOD_P = 1;
  public static final double HOOD_I = 0;
  public static final double HOOD_D = 0;
  public static final double HOOD_S = 1;
  public static final double HOOD_V = 0;
  public static final double HOOD_A = 0;
  public static final double HOOD_G = 0.01;

  public static final double HOOD_ZERO_ANGLE = 0.0615234375 - 0.0234375;

  // Distance from hub {Distance, angle}
  public static double[][] hoodLookUpTable = {
    {0.5, 0},
    {1, 0.490479},
    {1.5, 0.6}
    // {2, 0},
    // {2.5, 0},
    // {3, 0},
    // {3.5, 0},
    // {4, 0},
    // {4.5, 0},
    // {5, 0},
    // {5.5, 0},
    // {6, 0},
    // {6.5, 0},
    // {7, 0},
    // {7.5, 0},
    // {8, 0}
  };
}
