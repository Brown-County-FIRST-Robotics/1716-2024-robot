package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;

/** The abstraction for the CTRE Pigeon 2 IMU */
public class IMUIOPigeon implements IMUIO {
  Pigeon2 imu;

  /**
   * Constructs an IMU using a CAN id
   *
   * @param id The CAN id of the pigeon2
   */
  public IMUIOPigeon(int id) {
    imu = new Pigeon2(id);
    imu.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.tempC = imu.getTemperature().getValue();
    inputs.rotation = new Rotation3d(0, 0, imu.getRotation2d().getRadians());
    inputs.xAccelMPS = -1; // I don't think the pigeon has these. I really hope I'm wrong
    inputs.yAccelMPS = -1;
    inputs.zAccelMPS = -1;
  }
}
