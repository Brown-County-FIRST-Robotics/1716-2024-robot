package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** A command that finishes when the given condition has been true for the given duration */
public class ConditionTrueForDuration extends Command {
  private final double duration;
  private final BooleanSupplier condition;
  private final Timer timer = new Timer();

  /**
   * Creates a new ConditionTrueForDuration command
   *
   * @param duration The duration for which the condition must be true
   * @param condition The condition that must be true
   */
  public ConditionTrueForDuration(double duration, BooleanSupplier condition) {
    this.duration = duration;
    this.condition = condition;
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    if (!condition.getAsBoolean()) {
      timer.restart();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
