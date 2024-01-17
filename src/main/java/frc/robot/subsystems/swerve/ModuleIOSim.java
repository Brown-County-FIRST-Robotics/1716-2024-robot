package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveSimManager;

/** A simulated swerve module */
public class ModuleIOSim implements ModuleIO {
  private int index;

  /**
   * Constructs a ModuleIOSim given an index (fl:0,fr:1,bl:2,br:3)
   *
   * @param index The index of the module
   */
  public ModuleIOSim(int index) {
    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.pos = SwerveSimManager.getInstance().getModPos(index);
    inputs.vel = SwerveSimManager.getInstance().getModState(index);
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    SwerveModulePosition pos = SwerveSimManager.getInstance().getModPos(index);
    state.speedMetersPerSecond *= pos.angle.minus(state.angle).getCos();
    SwerveSimManager.getInstance().commandState(index, state);
  }
}
