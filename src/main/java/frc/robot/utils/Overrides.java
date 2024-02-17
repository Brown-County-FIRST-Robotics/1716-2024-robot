package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class Overrides {
  public static LoggedDashboardBoolean useFieldOriented =
      new LoggedDashboardBoolean("Use Field Oriented", false);
  public static LoggedDashboardBoolean resetYaw = new LoggedDashboardBoolean("Reset Yaw", false);
  public static LoggedDashboardBoolean disableIMU =
      new LoggedDashboardBoolean("Disable IMU", false);
  public static LoggedDashboardBoolean disableVision =
      new LoggedDashboardBoolean("Disable Vision", false);
  public static LoggedDashboardBoolean disableAutoAiming =
      new LoggedDashboardBoolean("Disable Auto Aiming", false);
  public static LoggedDashboardBoolean disableAutoAlign =
      new LoggedDashboardBoolean("Disable Auto Align", false);

  // intake from floor, intake from source, aim for amp, aim for speaker:
  public static LoggedDashboardBoolean disableArmAnglePresets =
      new LoggedDashboardBoolean("Disable Arm Angle Presets", false);
  public static LoggedTunableNumber armAngleOverrideIncrementScale =
      new LoggedTunableNumber("arm angle override increment scale", 1.0);
}
