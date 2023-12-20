package frc.robot;

public final class WhoAmI {
  public static final Mode mode = Mode.REAL;
  public static final RobotType bot = RobotType.SWERVEBASE;
  final Appendages[] appendages = {};

  public static enum RobotType {
    MECHBASE,
    SWERVEBASE,
    SIMSWERVEBASE
  }

  public static enum Appendages {}

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }
}
