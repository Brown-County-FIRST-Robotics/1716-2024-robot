package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** The arm IO layer */
public interface ArmIO {
  /** The inputs from the arm */
  @AutoLog
  class ArmIOInputs {
    /** The angle of the arm */
    Rotation2d angle = new Rotation2d();
    /** The arm velocity (rotations per second) */
    double omega = 0.0;
    /** The applied output of the motor (duty cycle) */
    double appliedOutput = 0.0;
    /** The temperature of the arm motor (Celsius) */
    double temperature = 0.0;
    double current=0.0;
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
