package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    Rotation2d angle = new Rotation2d();
    double omega = 0.0;
    double appliedOutput = 0.0;
    double temperature = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}
  ;

  default void setAngle(Rotation2d cmdAng, double arbFF) {}
  ;
}
