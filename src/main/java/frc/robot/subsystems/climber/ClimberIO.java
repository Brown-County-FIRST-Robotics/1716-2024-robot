package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    double leftPosition = 0;
    double rightPosition = 0;

    double leftSpeed = 0;
    double rightSpeed = 0;

    double leftTemp = 0;
    double rightTemp = 0;

    double leftOut = 0;
    double rightOut = 0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double leftVoltage, double rightVoltage) {}
}
