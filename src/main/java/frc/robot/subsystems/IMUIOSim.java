package frc.robot.subsystems;

import frc.robot.SwerveSimManager;

/** A simulated IMU */
public class IMUIOSim implements IMUIO {
  public IMUIOSim() {}

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.rotation = SwerveSimManager.getInstance().getIMUOutput();
  }
}
