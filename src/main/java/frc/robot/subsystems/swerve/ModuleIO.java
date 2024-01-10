package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

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

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setCmdState(SwerveModuleState state) {}
}
