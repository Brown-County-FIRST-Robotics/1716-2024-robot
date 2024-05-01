package frc.robot.utils;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/** Like a subsystem, but just the periodic function. Made for {@link LoggedTunableNumber}. */
public abstract class PeriodicRunnable {
  private static final ArrayList<PeriodicRunnable> allRunnable = new ArrayList<>();
  private static final ArrayList<PeriodicRunnable> proposedRunnable = new ArrayList<>();

  /** Runs the periodic method for all the instances of this class */
  public static void runPeriodic() {
    allRunnable.addAll(proposedRunnable);
    proposedRunnable.clear();
    for (PeriodicRunnable periodicRunnable : allRunnable) {
      periodicRunnable.periodic();
    }
    Logger.recordOutput(
        "PeriodicRunnable",
        allRunnable.stream().map(PeriodicRunnable::getName).toList().toArray(new String[] {}));
  }

  /** Constructs a new runnable */
  public PeriodicRunnable() {
    proposedRunnable.add(this);
  }

  /** Runs once every clock cycle (50hz) */
  public void periodic() {}

  public String getName() {
    return this.getClass().getName();
  }
}
