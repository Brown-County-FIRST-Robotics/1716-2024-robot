package frc.robot.subsystems.mecanum;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

/** The IO interface for mecanum motors */
public interface MecanumIO {
  /** Inputs from mecanum motors. Access using <code>MecanumIOInputsAutoLogged</code> */
  @AutoLog
  public static class MecanumIOInputs {
    /** Front left motor temperature in Celsius */
    double flTemp = 0;
    /** Front right motor temperature in Celsius */
    double frTemp = 0;
    /** Back left motor temperature in Celsius */
    double blTemp = 0;
    /** Back right motor temperature in Celsius */
    double brTemp = 0;

    /** Front left motor applied output */
    double flOut = 0;
    /** Front right motor applied output */
    double frOut = 0;
    /** Back left motor applied output */
    double blOut = 0;
    /** Back right motor applied output */
    double brOut = 0;
    /** The wheel speeds according to the encoders */
    MecanumDriveWheelSpeeds vel = new MecanumDriveWheelSpeeds();
    /** The encoder positions */
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
