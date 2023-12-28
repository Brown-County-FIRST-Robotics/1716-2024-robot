package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** The abstraction layer for the IMU */
public interface IMUIO {
  @AutoLog
  public static class IMUIOInputs {
    public double yaw = 0;
    public double roll = 0;
    public double pitch = 0;

    public double xAccelMPS = 0.0;
    public double yAccelMPS = 0.0;
    public double zAccelMPS = 0.0;
    public double tempC = 0.0;
  }

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs to update
   */
  public default void updateInputs(IMUIOInputs inputs) {}
}
