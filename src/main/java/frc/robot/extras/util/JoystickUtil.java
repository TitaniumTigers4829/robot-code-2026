package frc.robot.extras.util;

import frc.robot.Constants.JoystickConstants;
import java.util.function.DoubleSupplier;

public class JoystickUtil {

  /**
   * Deadbands a value to 0
   *
   * @param value Value to input
   * @param deadband If the the input is within this value, the function will return 0.
   * @return 0.0 if the value is within specified deadband range
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Function to modify a singular joystick axis.
   *
   * @param supplierValue Supplies a double, usually by a joystick.
   * @param exponent Exponent to raise the supplied value to.
   * @return Returns the modified value.
   */
  public static double modifyAxis(DoubleSupplier supplierValue, int exponent) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, JoystickConstants.DEADBAND_VALUE);

    // Raise value to specified exponent
    value = Math.copySign(Math.pow(value, exponent), value);

    return value;
  }

  /**
   * Converts the two axis coordinates to polar to get both the angle and radius, so they can work
   * in a double[] list.
   *
   * @param xJoystick The supplied input of the xJoystick.
   * @param yJoystick The supplied input of the yJoystick.
   * @param exponent The exponent to raise the supplied value to.
   * @return The modified axises of both joysticks in polar form.
   */
  public static double[] modifyAxisPolar(
      DoubleSupplier xJoystick, DoubleSupplier yJoystick, double exponent) {
    double xInput = deadband(xJoystick.getAsDouble(), JoystickConstants.DEADBAND_VALUE);
    double yInput = deadband(yJoystick.getAsDouble(), JoystickConstants.DEADBAND_VALUE);
    if (Math.abs(xInput) > 0 && Math.abs(yInput) > 0) {
      double theta = Math.atan(xInput / yInput);
      double hypotenuse = Math.sqrt(xInput * xInput + yInput * yInput);
      double raisedHypotenuse = Math.pow(hypotenuse, exponent);
      xInput = Math.copySign(Math.sin(theta) * raisedHypotenuse, xInput);
      yInput = Math.copySign(Math.cos(theta) * raisedHypotenuse, yInput);
      return new double[] {xInput, yInput};
    }
    return new double[] {
      Math.copySign(Math.pow(xInput, exponent), xInput),
      Math.copySign(Math.pow(yInput, exponent), yInput)
    };
  }
}
