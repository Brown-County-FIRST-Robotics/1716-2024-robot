package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    boolean beamBroke = false;
    double motorTemperature = 0.0;
    double motorCurrent = 0.0;
    double motorOutput = 0.0;
    double position = 0.0;
    double velocity = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVoltage(double voltage) {}
}
