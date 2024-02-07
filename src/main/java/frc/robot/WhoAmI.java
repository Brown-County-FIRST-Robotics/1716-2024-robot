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
    ARM,
    SIM_ARM,
    SHOOTER,
    SIM_SHOOTER,
    CLIMBER,
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
