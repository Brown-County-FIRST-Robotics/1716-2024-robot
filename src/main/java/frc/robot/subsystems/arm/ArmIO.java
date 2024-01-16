package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    Rotation2d angle = new Rotation2d();
    double omega = 0.0;
    double appliedOutput = 0.0;
    double temperature = 0.0;
  }
  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs to update
   */
  default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Commands an angle
   *
   * @param cmdAng The command angle
   * @param arbFF The arbitrary feedforward for gravity compensation (in volts)
   */
  default void setAngle(Rotation2d cmdAng, double arbFF) {}
}
