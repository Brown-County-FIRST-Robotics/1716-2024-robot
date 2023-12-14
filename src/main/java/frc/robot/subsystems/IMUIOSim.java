package frc.robot.subsystems;

import frc.robot.SwerveSimManager;

public class IMUIOSim implements IMUIO {
  public IMUIOSim() {}

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.yaw = SwerveSimManager.getInstance().getIMUOutput().getDegrees();
  }
}
