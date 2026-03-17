package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;

public class PhysicalGyroPigeon implements GyroInterface {

  private final Pigeon2 gyro =
      new Pigeon2(DriveConstants.PIGEON_ID, HardwareConstants.CANIVORE_CAN_BUS_STRING);

  // TODO: Add BaseStatusSignal list
  private final StatusSignal<Angle> yaw = gyro.getYaw();
  private final StatusSignal<Angle> pitch = gyro.getPitch();
  private final StatusSignal<Angle> roll = gyro.getRoll();
  private final StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityZWorld();
  private final StatusSignal<AngularVelocity> pitchVelocity = gyro.getAngularVelocityXWorld();
  private final StatusSignal<AngularVelocity> rollVelocity = gyro.getAngularVelocityYWorld();
  private final StatusSignal<LinearAcceleration> xAccel = gyro.getAccelerationX();
  private final StatusSignal<LinearAcceleration> yAccel = gyro.getAccelerationY();

  public PhysicalGyroPigeon() {
    gyro.getConfigurator().apply(new Pigeon2Configuration());

    gyro.getConfigurator().setYaw(0.0);

    // We care most about the yaw and yaw velocity from the gyro, so we set it to have a higher
    // frequency than other
    // signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.CANIVORE_SIGNAL_FREQUENCY, yaw, yawVelocity);

    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.RIO_SIGNAL_FREQUENCY,
        pitch,
        roll,
        pitchVelocity,
        rollVelocity,
        xAccel,
        yAccel);
    gyro.setYaw(0.0);
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    BaseStatusSignal.refreshAll(
        yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity, xAccel, yAccel);
    // yaw.refresh();
    // pitch.refresh();
    // roll.refresh();
    // yawVelocity.refresh();
    // pitchVelocity.refresh();
    // rollVelocity.refresh();
    // xAccel.refresh();
    // yAccel.refresh();
    inputs.isConnected = gyro.isConnected();
    inputs.yawVelocityDegreesPerSecond = yawVelocity.getValueAsDouble();
    inputs.rollVelocityDegreesPerSecond = rollVelocity.getValueAsDouble();
    inputs.pitchVelocityDegreesPerSecond = pitchVelocity.getValueAsDouble();
    inputs.yawDegrees = yaw.getValueAsDouble();
    inputs.accelX = xAccel.getValueAsDouble();
    inputs.accelY = yAccel.getValueAsDouble();
    inputs.rollDegrees = roll.getValueAsDouble();
    inputs.pitchDegrees = pitch.getValueAsDouble();
  }

  @Override
  public void reset() {
    gyro.reset();
  }
}
