package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

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
    // TEMP: add real implementation
    DriverStation.reportWarning("Simulation is currently broken, will fix soon?", false);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // TEMP: add real implementation
    DriverStation.reportWarning("Simulation is currently broken, will fix soon?", false);
  }

  @Override
  public void setCmdState(double ang, double vel) {
    // TEMP: add real implementation
    DriverStation.reportWarning("Simulation is currently broken, will fix soon?", false);
  }
}
