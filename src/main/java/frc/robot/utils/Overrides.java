package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class Overrides {
  public static LoggedDashboardBoolean useFieldOriented =
      new LoggedDashboardBoolean("Use Field Oriented", true);
  public static LoggedDashboardBoolean resetYaw = new LoggedDashboardBoolean("Reset Yaw", false);
  public static LoggedDashboardBoolean disableIMU =
      new LoggedDashboardBoolean("Disable IMU", false);
  public static LoggedDashboardBoolean disableVision =
      new LoggedDashboardBoolean("Disable Vision", true);
  public static LoggedDashboardBoolean disableAutoAiming =
      new LoggedDashboardBoolean("Disable Auto Aiming", true);
  public static LoggedDashboardBoolean disableAutoAlign =
      new LoggedDashboardBoolean("Disable Auto Align", true);

  // intake from floor, intake from source, aim for amp, aim for speaker:
  public static LoggedDashboardBoolean disableArmAnglePresets =
      new LoggedDashboardBoolean("Disable Arm Angle Presets", false);
  public static LoggedTunableNumber armAngleOverrideIncrementScale =
      new LoggedTunableNumber("arm angle override increment scale", 1.0);
public static LoggedTunableNumber kitbot=new LoggedTunableNumber("kitbot",60);
}
