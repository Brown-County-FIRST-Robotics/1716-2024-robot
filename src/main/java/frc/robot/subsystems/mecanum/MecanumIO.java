package frc.robot.subsystems.mecanum;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface MecanumIO {
  @AutoLog
  public static class MecanumIOInputs{
    MecanumDriveWheelPositions pos=new MecanumDriveWheelPositions();
    double flTemp=0;
    double frTemp=0;
    double blTemp=0;
    double brTemp=0;
  }

  public default void setSpeeds(MecanumDriveWheelSpeeds cmd) {}
  public default void reconfigure(){}
  public default void updateInputs(MecanumIOInputs inputs) {}
}
