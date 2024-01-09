package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/** A simple command to retrieve the desired command at runtime */
public class SuppliedCommand extends CommandBase {

  private final Supplier<Command> supplier;
  private Command cmd;

  /**
   * Constructs a new SuppliedCommand
   *
   * @param supplier The command supplier to use
   * @param requirements The subsystem requirements
   */
  public SuppliedCommand(Supplier<Command> supplier, Subsystem... requirements) {
    addRequirements(requirements);
    this.supplier = supplier;
  }

  @Override
  public void initialize() {
    cmd = supplier.get();
    cmd.initialize();
  }

  @Override
  public void execute() {
    cmd.execute();
  }

  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    cmd.end(interrupted);
  }
}
