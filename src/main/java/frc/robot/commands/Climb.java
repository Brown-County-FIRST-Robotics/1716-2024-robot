package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class Climb extends Command {
  private final Climber climber;
  private final DoubleSupplier roll;

  public Climb(Climber climber, DoubleSupplier roll) {
    this.climber = climber;
    this.roll = roll;
    addRequirements(climber);
  }

  @Override
  public void execute() {}
}
