package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    double leftPosition = 0;
    double rightPosition = 0;

    double leftSpeed = 0;
    double rightSpeed = 0;

    double leftTemp = 0;
    double rightTemp = 0;

    double leftOut = 0;
    double rightOut = 0;

    boolean leftBottomSensor = false;
    boolean leftTopSensor = false;
    boolean rightBottomSensor = false;
    boolean rightTopSensor = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setMotors(double left, double right) {}

  default void setMotorEncoderPosition(boolean setRight, double position) {}
}
