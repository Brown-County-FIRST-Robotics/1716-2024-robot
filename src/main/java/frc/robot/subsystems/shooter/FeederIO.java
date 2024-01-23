package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederPos = 0.0;
  }

  default void updateInputs(FeederIOInputs inputs) {}

  default void cmdPos(double pos) {}

  default void resetPos() {}
}
