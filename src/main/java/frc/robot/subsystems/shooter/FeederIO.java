package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** IO layer for the feeder */
public interface FeederIO {
  /**
   * Sets the commanded speed of the feeder motor
   *
   * @param vel The feeder speed in RPM
   */
  default void setVel(double vel) {}

  /** The inputs from the feeder. Access using <code>FeederIOInputsAutoLogged</code> */
  @AutoLog
  class FeederIOInputs {
    /** The position of the feeder motor */
    public double position = 0.0;
    /** The velocity of the feeder motor */
    public double velocity = 0.0;
    /** The current draw of the feeder motor */
    public double current = 0.0;
    /** The output of the feeder motor */
    public double appliedOutput = 0.0;
    /** The temperature of the feeder motor */
    public double temperature = 0.0;
    /** The value of the open contact on the limit switch */
    public boolean openContact = false;
    /** The value of the closed contact on the limit switch */
    public boolean closedContact = true;
  }

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  default void updateInputs(FeederIOInputs inputs) {}
}
