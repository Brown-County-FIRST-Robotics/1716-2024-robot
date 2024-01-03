package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveSimManager;

public class ModuleIOSim implements ModuleIO {
  private int id;

  public ModuleIOSim(int id) {
    this.id = id;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    SwerveModulePosition pos = SwerveSimManager.getInstance().getModPos(id);
    inputs.steerPos = pos.angle.getRotations();
    inputs.thrustPos = pos.distanceMeters;
    inputs.thrustVel = SwerveSimManager.getInstance().getModState(id).speedMetersPerSecond;
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    SwerveModulePosition pos = SwerveSimManager.getInstance().getModPos(id);
    state.speedMetersPerSecond *= pos.angle.minus(state.angle).getCos();
    SwerveSimManager.getInstance().commandState(id, state);
  }
}
