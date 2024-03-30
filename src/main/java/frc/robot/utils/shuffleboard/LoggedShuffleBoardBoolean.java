package frc.robot.utils.shuffleboard;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import java.util.ArrayList;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedShuffleBoardBoolean implements LoggedDashboardInput {
  @AutoLog
  public static class LoggedShuffleBoardBooleanInputs {
    public boolean val = false;
  }

  private final String key;
  private SimpleWidget widget;
  private BooleanEntry entry;
  private LoggedShuffleBoardBooleanInputsAutoLogged inputs =
      new LoggedShuffleBoardBooleanInputsAutoLogged();
  private final ArrayList<Consumer<Boolean>> listeners = new ArrayList<>();
  private boolean lastVal = false;

  public LoggedShuffleBoardBoolean(String tab, String key) {
    this.key = key;
    widget = Shuffleboard.getTab(tab).add(key, false);
    entry = (BooleanEntry) widget.getEntry();

    periodic();
    Logger.registerDashboardInput(this);
  }

  public boolean get() {
    return inputs.val;
  }

  public void attach(Consumer<Boolean> listener) {
    listeners.add(listener);
    listener.accept(get());
  }

  public void set(boolean val) {
    if (!Logger.hasReplaySource()) {
      entry.set(val);
    }
  }

  @Override
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      inputs.val = entry.get(false);
    }
    Logger.processInputs("DashboardInputs/" + key, inputs);
    if (lastVal == inputs.val) {
      listeners.forEach((listener) -> listener.accept(get()));
      lastVal = inputs.val;
    }
  }
}
