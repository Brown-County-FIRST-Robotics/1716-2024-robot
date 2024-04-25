package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class Overrides {
  public static final LoggedDashboardBoolean useFieldOriented =
      new LoggedDashboardBoolean("Use Field Oriented", true);
  public static final LoggedDashboardBoolean resetYaw =
      new LoggedDashboardBoolean("Reset Yaw", false);
  public static final LoggedDashboardBoolean disableIMU =
      new LoggedDashboardBoolean("Disable IMU", false);
  public static final LoggedDashboardBoolean disableVision =
      new LoggedDashboardBoolean("Disable Vision", false);
  public static final LoggedDashboardBoolean disableAutoAiming =
      new LoggedDashboardBoolean("Disable Auto Aiming", false);
  public static final LoggedDashboardBoolean disableAutoAlign =
      new LoggedDashboardBoolean("Disable Auto Align", false);

  // intake from floor, intake from source, aim for amp, aim for speaker:
  public static final LoggedDashboardBoolean disableArmAnglePresets =
      new LoggedDashboardBoolean("Disable Arm Angle Presets", false);
  public static final LoggedTunableNumber armAngleOverrideIncrementScale =
      new LoggedTunableNumber("arm angle override increment scale", 0.1);
  public static final LoggedTunableNumber kitbot = new LoggedTunableNumber("kitbot", 60);
}
