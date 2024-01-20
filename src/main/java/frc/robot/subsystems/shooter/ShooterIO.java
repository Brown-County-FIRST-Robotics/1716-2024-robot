package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  void setVelocity(double vel);

  @AutoLog
  public static class ShooterIOInputs {
    boolean beamBroke = false;
    double[] motorTemperature = new double[]{};
    double[] motorCurrent = new double[]{};
    double[] motorOutput = new double[]{};
    double[] position = new double[]{};
    double[] velocity = new double[]{};
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVoltage(double voltage) {}
}
