package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import org.littletonrobotics.junction.Logger;

public class IMUIONavx implements IMUIO {
  AHRS imu;

  public IMUIONavx() {
    imu = new AHRS(SPI.Port.kMXP);
    while(!imu.isConnected()){
      imu.isConnected();
    }
  }

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.tempC = imu.getTempC();
    inputs.pitch = imu.getPitch();
    inputs.yaw = -imu.getYaw();
    inputs.roll = imu.getRoll();
    inputs.xAccelMPS = imu.getRawAccelX() * 9.8065;
    inputs.yAccelMPS = imu.getRawAccelY() * 9.8065;
    inputs.zAccelMPS = imu.getRawAccelZ() * 9.8065;
  }
}