package frc.robot.utils;

import java.util.ArrayList;

/** Like a subsystem, but just the periodic function. Made for {@link LoggedTunableNumber}. */
public abstract class PeriodicRunnable {
  private static ArrayList<PeriodicRunnable> allRunnable = new ArrayList<>();

  /** Runs the periodic method for all the instances of this class */
  public static void runPeriodic() {
    for (PeriodicRunnable periodicRunnable : allRunnable) {
      periodicRunnable.periodic();
    }
  }

  public PeriodicRunnable() {
    allRunnable.add(this);
  }

  /** Runs once every clock cycle (50hz) */
  public void periodic() {}
}
