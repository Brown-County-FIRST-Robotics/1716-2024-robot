package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  default void setVel(double vel) {}

  @AutoLog
  public static class FeederIOInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double current = 0.0;
    public boolean openContact = false;
    public boolean closedContact = true;
  }

  default void updateInputs(FeederIOInputs inputs) {}
}
