// controls both sides of the climber with the same stick and levels the bot
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ClimbAndLevel extends Command {
  private final Climber climber;
  private final DoubleSupplier movement;
  private final DoubleSupplier roll;

  double leftPercent;
  double rightPercent;

  LoggedTunableNumber deadzone = new LoggedTunableNumber("Climber/Deadzone", 0.05);

  public ClimbAndLevel(Climber climber, DoubleSupplier movement, DoubleSupplier roll) {
    this.climber = climber;
    this.movement = movement;
    this.roll = roll;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    leftPercent = (movement.getAsDouble() + levelPercentModifier(false)) * 0.75;
    leftPercent = (movement.getAsDouble() + levelPercentModifier(true)) * 0.75;

    if (Math.abs(leftPercent) < deadzone.get() && Math.abs(rightPercent) < deadzone.get()) {
      climber.setMotors(0, 0);
    } else {
      climber.setMotors(leftPercent, rightPercent);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.setMotors(0, 0);
  }

  // takes the roll of the robot and returns a modifier in volts
  private double levelPercentModifier(boolean rightSide) {
    double roll = this.roll.getAsDouble();
    if (rightSide) { // TODO: DETERMINE IF THIS IS THE CORRECT SIDE
      roll = -roll;
    }

    // https://www.desmos.com/calculator/4lzmqs1sj1
    double modifier = Math.pow(2.0, roll); // TODO: MANIPULATE THIS FUNCTION TO WORK BETTER

    if (!rightSide) { // TODO: DETERMINE IF THIS IS THE CORRECT SIDE
      return modifier;
    } else {
      return -modifier;
    }
  }
}
