package frc.robot;

public final class WhoAmI {
  final Mode mode=Mode.REAL;
  final RobotType bot=RobotType.SWERVEBASE;
  final Appendages[] appendages={};
  public static enum RobotType {
    MECHBASE, SWERVEBASE
  }

  public static enum Appendages {

  }

  public static enum Mode {
    REAL, REPLAY, SIM
  }
}
