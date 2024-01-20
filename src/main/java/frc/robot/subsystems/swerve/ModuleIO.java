package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/** The IO layer for a swerve module */
public interface ModuleIO {
  /** The inputs to the code from a swerve module */
  @AutoLog
  public static class ModuleIOInputs {
    /** The velocity according to the encoders */
    public SwerveModuleState vel = new SwerveModuleState();
    /** The position according to the encoders */
    public SwerveModulePosition pos = new SwerveModulePosition();
    /** The temperature of the steer motor in Celsius */
    public double steerTempC = 0.0;
    /** The temperature of the thrust motor in Celsius */
    public double thrustTempC = 0.0;
    /** The closed-loop error of the thrust motor */
    public double thrustErr = 0.0;
    /** The swerve module offset */
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
