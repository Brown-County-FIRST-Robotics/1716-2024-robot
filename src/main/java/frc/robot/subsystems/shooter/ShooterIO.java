package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** The inputs from the shooter. Access using <code>ShooterIOInputsAutoLogged</code> */
  @AutoLog
  class ShooterIOInputs {
    double[] motorTemperature = new double[] {0, 0};
    double[] motorCurrent = new double[] {0, 0};
    double[] motorOutput = new double[] {0, 0};
    double[] position = new double[] {0, 0};
    double[] velocity = new double[] {0, 0};
  }
  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVelocity(double vel1, double vel2) {}
}
