package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double steerPos =
        0.0; // Advantagekit currently doesn't currently support SwerveModulePosition here, so we
    // use an intermediate form
    public double thrustPos =
        0.0; // Advantagekit currently doesn't currently support SwerveModulePosition here, so we
    // use an intermediate form
    public double thrustVel = 0.0;
    public double steerTempC = 0.0;
    public double thrustTempC = 0.0;
    public double thrustErr = 0.0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}
  ;

  public default void setCmdState(SwerveModuleState state) {}
  ;

  public default void reconfigure() {}
  ;
}
