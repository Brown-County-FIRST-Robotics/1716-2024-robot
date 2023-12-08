package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import org.littletonrobotics.junction.Logger;

public class IMUIOPigeon implements IMUIO{
  WPI_Pigeon2 imu;

  public IMUIOPigeon(int id) {
    imu=new WPI_Pigeon2(id);
    Logger.getInstance().recordMetadata("IMUFW", String.valueOf(imu.getFirmwareVersion()));
  }

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.tempC=imu.getTemp();
    inputs.yaw=imu.getRotation2d().getDegrees();
    inputs.pitch=imu.getPitch();
    inputs.roll=imu.getRoll();
    inputs.xAccelMPS=-1;// I don't think the pigeon has these. I really hope I'm wrong
    inputs.yAccelMPS=-1;
    inputs.zAccelMPS=-1;
  }
}
