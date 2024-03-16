package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedShuffleBoardChooser<V> implements LoggedDashboardInput {
  @AutoLog
  public static class LoggedShuffleBoardChooserInputs {
    public String selectedVal = "";
  }

  private final String key;
  private final SendableChooser<String> sendableChooser;
  private Map<String, V> options = new HashMap<>();
  private LoggedShuffleBoardChooserInputsAutoLogged inputs =
      new LoggedShuffleBoardChooserInputsAutoLogged();
  private final ArrayList<Consumer<V>> listeners = new ArrayList<>();
  private String lastVal = "";

  public LoggedShuffleBoardChooser(String tab, String key) {
    this.key = key;
    sendableChooser = new SendableChooser<>();
    Shuffleboard.getTab(tab).add(key, sendableChooser);
    periodic();
    Logger.registerDashboardInput(this);
  }

  public void addOption(String key, V value) {
    sendableChooser.addOption(key, key);
    options.put(key, value);
  }

  public void addDefaultOption(String key, V value) {
    sendableChooser.setDefaultOption(key, key);
    options.put(key, value);
  }

  public V get() {
    return options.get(inputs.selectedVal);
  }

  public void attach(Consumer<V> listener) {
    listeners.add(listener);
    if (inputs.selectedVal != null) {
      if (!inputs.selectedVal.isEmpty()) {
        listener.accept(get());
      }
    }
  }

  @Override
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      inputs.selectedVal = sendableChooser.getSelected();
    }

    Logger.processInputs("DashboardInputs/" + key, inputs);
    if (inputs.selectedVal != null) {
      if (!Objects.equals(inputs.selectedVal, lastVal) && !inputs.selectedVal.isEmpty()) {
        listeners.forEach((listener) -> listener.accept(get()));
        lastVal = inputs.selectedVal;
      }
    }
  }
}
