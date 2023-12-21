package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class LoggedTunableNumber {
  private static final String tableKey = "Tuning";
  private final String key;
  private LoggedDashboardNumber dashboardNumber;
  private double defaultValue;
  private boolean hasDefault;
  private double lastHasChangedValue=-1;

  public LoggedTunableNumber(String name) {
    key = tableKey + "/" + name;
  }

  public LoggedTunableNumber(String name, double defaultValue) {
    this(name);
    initDefault(defaultValue);
  }

  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      dashboardNumber = new LoggedDashboardNumber(key, defaultValue);
    }
  }

  public double get() {
    return hasDefault ? dashboardNumber.get() : 0.0;
  }

  public boolean hasChanged() {
    double currentVal = get();
    if (currentVal != lastHasChangedValue) {
      lastHasChangedValue = currentVal;
      return true;
    }
    return false;
  }
}
