// Split climbing for individual control of the climber sides
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ClimbSplit extends Command {
  private final Climber climber;
  private final DoubleSupplier leftMovement;
  private final DoubleSupplier rightMovement;

  double leftPercent;
  double rightPercent;

  LoggedTunableNumber deadzone = new LoggedTunableNumber("Climber/Deadzone", 0.05);

  public ClimbSplit(Climber climber, DoubleSupplier leftMovement, DoubleSupplier rightMovement) {
    this.climber = climber;
    this.leftMovement = leftMovement;
    this.rightMovement = rightMovement;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    leftPercent = leftMovement.getAsDouble() * 0.75;
    rightPercent = rightMovement.getAsDouble() * 0.75;

    if (Math.abs(leftPercent) < deadzone.get()) {
      leftPercent = 0;
    }
    if (Math.abs(rightPercent) < deadzone.get()) {
      rightPercent = 0;
    }

    climber.setMotors(leftPercent, rightPercent);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotors(0, 0);
  }
}
