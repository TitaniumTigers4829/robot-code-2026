package frc.robot.subsystems.adjustableHood;

public class AdjustableHoodConstants {
  public static final int HOOD_MOTOR_ID = 30;
  public static final int CANCODER_ID = 50;

  // public static final double HOOD_MAX_ANGLE = 0.5;
  public static final double GEAR_RATIO = 24 / 42;

  // Tune G first, increase until hood moves then tune pid and sva
  public static final double HOOD_P = 75; // 60
  public static final double HOOD_I = 0.0;
  public static final double HOOD_D = 0.0;
  public static final double HOOD_S = 0.23;
  public static final double HOOD_V = 0.0; // 0.08
  public static final double HOOD_A = 0;
  public static final double HOOD_G = 0;

  public static final double HOOD_ZERO_ANGLE = 0.199707;

  // Distance from hub {Distance (meters), angle}
  public static double[][] hoodLookUpTable = {
    {1.5, .25},
    {2.0, .3},
    {2.5, 0.347},
    {3.0, .38},
    {3.5, .4829},
    {4.0, 0.55},
    {4.5, 0.59}

    // 2.5: 45:0.4
    // TODO: scuff
    // 2.6: 45:0.4
    // 2.8: 50:0.5
    // 3: 53:0.6
    // 4.1: 60:0.65
    // 188" (5.3m): 75:0.75

  };
}
