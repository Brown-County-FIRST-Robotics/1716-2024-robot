package frc.robot.subsystems.mecanum;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface MecanumIO {
  @AutoLog
  public static class MecanumIOInputs {
    double flTemp = 0;
    double frTemp = 0;
    double blTemp = 0;
    double brTemp = 0;
    double flOut = 0;
    double frOut = 0;
    double blOut = 0;
    double brOut = 0;
    MecanumDriveWheelSpeeds vel = new MecanumDriveWheelSpeeds();
    MecanumDriveWheelPositions pos = new MecanumDriveWheelPositions();
  }

  /**
   * Command a speed to the motors
   *
   * @param cmd The commanded speeds
   */
  public default void setSpeeds(MecanumDriveWheelSpeeds cmd) {}

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  public default void updateInputs(MecanumIOInputs inputs) {}
}
