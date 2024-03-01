package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Climb extends Command {
  private final Climber climber;
  private final DoubleSupplier movement;
  private final DoubleSupplier roll;

  double leftVoltage;
  double rightVoltage;

  LoggedTunableNumber voltageDeadzone = new LoggedTunableNumber("Climber/Voltage Deadzone", 0.1);

  public Climb(Climber climber, DoubleSupplier movement, DoubleSupplier roll) {
    this.climber = climber;
    this.movement = movement;
    this.roll = roll;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    leftVoltage = movement.getAsDouble() + levelVoltageModifier(false);
    rightVoltage = movement.getAsDouble() + levelVoltageModifier(true);

    if (leftVoltage < voltageDeadzone.get() && rightVoltage < voltageDeadzone.get()) {
      climber.setVoltage(0, 0);
    } else {
      climber.setVoltage(leftVoltage, rightVoltage);
    }
    climber.setVoltage(1.0, 1);
  }

  @Override
  public void end(boolean interrupted) {
    climber.setVoltage(0, 0);
  }

  // takes the roll of the robot and returns a modifier in volts
  private double levelVoltageModifier(boolean rightSide) {
    double roll = this.roll.getAsDouble();
    if (rightSide) { // TODO: DETERMINE IF THIS IS THE CORRECT SIDE
      roll = -roll;
    }

    // https://www.desmos.com/calculator/4lzmqs1sj1
    double voltage = Math.pow(2.0, roll); // TODO: MANIPULATE THIS FUNCTION TO WORK BETTER

    if (!rightSide) { // TODO: DETERMINE IF THIS IS THE CORRECT SIDE
      return voltage;
    } else {
      return -voltage;
    }
  }
}
