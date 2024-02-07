package frc.robot;

/** The class that contains the information about whom the robot is */
public final class WhoAmI {
  /** The mode of the robot */
  public static final Mode mode = Mode.REAL;
  /** The robot */
  public static final RobotType bot = RobotType.SWERVEBASE;
  /** The appendages to the robot */
  public static final Appendages[] appendages = {};

  /** The robot types */
  public static enum RobotType {
    /** The mecanum robot */
    MECHBASE,
    /** A simulated swerve robot */
    SIMSWERVEBASE,
    /** The swerve robot */
    SWERVEBASE
  }

  /** The appendages to the robot */
  public static enum Appendages {
    /** An arm that pivots the shooter */
    ARM,
    /** A simulated arm that pivots the shooter */
    SIM_ARM,
    /** A shooter for notes */
    SHOOTER,
    /** A simulated shooter for notes */
    SIM_SHOOTER,
    /** A climber for the chain */
    CLIMBER,
    /** A simulated climber for the chain */
    SIM_CLIMBER,
  }

  /** The code execution mode */
  public static enum Mode {
    /** A real robot */
    REAL,
    /** Log file replay */
    REPLAY,
    /** Simulated */
    SIM
  }

  /**
   * Checks the configuration
   *
   * @param args Not used
   */
  public static void main(String... args) {
    if (mode != Mode.REAL) {
      throw new IllegalArgumentException("Cannot deploy code in Sim mode to the robot");
    }
    boolean override = false; // Make true to override deploy checking
    if (bot == RobotType.SIMSWERVEBASE && !override) {
      throw new IllegalArgumentException(
          "You are currently deploying code meant for the simulator to a real robot. ONLY DO THIS IF YOU ABSOLUTELY KNOW WHAT YOU ARE DOING. ");
    }
  }
}
