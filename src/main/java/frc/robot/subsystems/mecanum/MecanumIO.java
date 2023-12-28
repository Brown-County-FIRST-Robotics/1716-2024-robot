package frc.robot.subsystems.mecanum;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface MecanumIO {
  @AutoLog
  public static class MecanumIOInputs {
    double flTemp = 0;
    double flPos = 0;
    double frTemp = 0;
    double frPos = 0;
    double blTemp = 0;
    double blPos = 0;
    double brTemp = 0;
    double brPos = 0;
    double flOut = 0;
    double flVel = 0;
    double frOut = 0;
    double frVel = 0;
    double blOut = 0;
    double blVel = 0;
    double brOut = 0;
    double brVel = 0;
  }

  public default void setSpeeds(MecanumDriveWheelSpeeds cmd) {}

  public default void updateInputs(MecanumIOInputs inputs) {}
}
