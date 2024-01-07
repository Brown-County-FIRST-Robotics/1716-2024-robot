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
    inputs.pos = SwerveSimManager.getInstance().getModPos(id);
    inputs.vel = SwerveSimManager.getInstance().getModState(id);
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    SwerveModulePosition pos = SwerveSimManager.getInstance().getModPos(id);
    state.speedMetersPerSecond *= pos.angle.minus(state.angle).getCos();
    SwerveSimManager.getInstance().commandState(id, state);
  }
}
