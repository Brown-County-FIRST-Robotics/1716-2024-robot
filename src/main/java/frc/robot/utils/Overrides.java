package frc.robot.utils;

import frc.robot.utils.shuffleboard.LoggedShuffleBoardBoolean;

public class Overrides {
  public static LoggedShuffleBoardBoolean useFieldOriented =
      new LoggedShuffleBoardBoolean("Teleop", "Use Field Oriented", true);
  public static LoggedShuffleBoardBoolean resetYaw =
      new LoggedShuffleBoardBoolean("Teleop", "Reset Yaw", false);
  public static LoggedShuffleBoardBoolean disableIMU =
      new LoggedShuffleBoardBoolean("Teleop", "Disable IMU", false);
  public static LoggedShuffleBoardBoolean disableVision =
      new LoggedShuffleBoardBoolean("Teleop", "Disable Vision", false);
  public static LoggedShuffleBoardBoolean disableAutoAiming =
      new LoggedShuffleBoardBoolean("Teleop", "Disable Auto Aiming", false);
  public static LoggedShuffleBoardBoolean disableAutoAlign =
      new LoggedShuffleBoardBoolean("Teleop", "Disable Auto Align", false);

  // intake from floor, intake from source, aim for amp, aim for speaker:
  public static LoggedShuffleBoardBoolean disableArmAnglePresets =
      new LoggedShuffleBoardBoolean("Teleop", "Disable Arm Angle Presets", false);
  public static LoggedTunableNumber armAngleOverrideIncrementScale =
      new LoggedTunableNumber("arm angle override increment scale", 1.0);
  public static LoggedTunableNumber kitbot = new LoggedTunableNumber("kitbot", 60);
}
