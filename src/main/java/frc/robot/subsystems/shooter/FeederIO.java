package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double current = 0.0;
  }

  default void updateInputs(FeederIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void resetPos() {}
}
