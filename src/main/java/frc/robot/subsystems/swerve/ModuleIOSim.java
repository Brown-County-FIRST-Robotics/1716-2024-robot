package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveSimManager;

/** A simulated swerve module */
public class ModuleIOSim implements ModuleIO {
  private final int index;
  private static final Rotation2d[] chassisOffsets =
      new Rotation2d[] {
        Rotation2d.fromDegrees(-90),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(90)
      };

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
    var realAng =
        SwerveSimManager.getInstance()
            .getModPos(index)
            .angle
            .plus(chassisOffsets[index])
            .unaryMinus();
    inputs.pos =
        new SwerveModulePosition(
            SwerveSimManager.getInstance().getModPos(index).distanceMeters, realAng);
    inputs.vel =
        new SwerveModuleState(
            SwerveSimManager.getInstance().getModState(index).speedMetersPerSecond, realAng);
    inputs.offset = 0;
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    var realRot = state.angle.unaryMinus().minus(chassisOffsets[index]);
    SwerveSimManager.getInstance()
        .commandState(index, new SwerveModuleState(state.speedMetersPerSecond, realRot));
  }
}
