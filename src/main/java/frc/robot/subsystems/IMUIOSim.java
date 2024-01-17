package frc.robot.subsystems;

import frc.robot.SwerveSimManager;

/** A simulated IMU */
public class IMUIOSim implements IMUIO {
  /** Constructs a new IMUIOSim */
  public IMUIOSim() {}

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.rotation = SwerveSimManager.getInstance().getIMUOutput();
  }
}
