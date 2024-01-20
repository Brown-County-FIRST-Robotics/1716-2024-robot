package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  void setVelocity(double vel);

  @AutoLog
  public static class ShooterIOInputs {
    boolean beamBroke = false;
    double[] motorTemperature = new double[]{0,0};
    double[] motorCurrent = new double[]{0,0};
    double[] motorOutput = new double[]{0,0};
    double[] position = new double[]{0,0};
    double[] velocity = new double[]{0,0};
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVoltage(double voltage) {}
}
