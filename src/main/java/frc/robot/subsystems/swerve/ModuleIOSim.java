package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveSimManager;

public class ModuleIOSim implements ModuleIO {
  private int id;
  private static final Rotation2d[] chassisOffsets =
      new Rotation2d[] {
        Rotation2d.fromDegrees(-90),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(90)
      };

  public ModuleIOSim(int id) {
    this.id = id;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    var realAng =
        SwerveSimManager.getInstance().getModPos(id).angle.plus(chassisOffsets[id]).unaryMinus();
    inputs.pos =
        new SwerveModulePosition(
            SwerveSimManager.getInstance().getModPos(id).distanceMeters, realAng);
    inputs.vel =
        new SwerveModuleState(
            SwerveSimManager.getInstance().getModState(id).speedMetersPerSecond, realAng);
    inputs.offset = 0;
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    var realRot = state.angle.unaryMinus().minus(chassisOffsets[id]);
    SwerveSimManager.getInstance()
        .commandState(id, new SwerveModuleState(state.speedMetersPerSecond, realRot));
  }
}
