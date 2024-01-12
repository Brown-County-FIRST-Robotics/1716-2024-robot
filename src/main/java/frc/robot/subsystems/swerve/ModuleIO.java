package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/** The IO layer for a swerve module */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public SwerveModuleState vel = new SwerveModuleState();
    public SwerveModulePosition pos = new SwerveModulePosition();
    public double steerTempC = 0.0;
    public double thrustTempC = 0.0;
    public double thrustErr = 0.0;
    public double offset = 0.0;
  }
  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Commands the module state
   *
   * @param state The state to command
   */
  public default void setCmdState(SwerveModuleState state) {}
}
