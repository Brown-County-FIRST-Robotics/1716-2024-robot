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
    public void execute() {
        climber.setVoltage(voltageFunction(false), voltageFunction(true));
    }

    @Override
    public void end(boolean interrupted) {
        climber.setVoltage(0, 0);
    }

    //takes the roll of the function and returns the voltage of the motor
    private double voltageFunction(boolean rightSide) {
        double roll = this.roll.getAsDouble();
        if (rightSide) { //TODO: DETERMINE IF THIS IS THE CORRECT SIDE
            roll = -roll;
        }

        //https://www.desmos.com/calculator/4lzmqs1sj1
        double voltage = Math.pow(2.0, roll); //TODO: MANIPULATE THIS FUNCTION TO WORK BETTER

        if (!rightSide) { //TODO: DETERMINE IF THIS IS THE CORRECT SIDE
            return voltage;
        }
        else {
            return -voltage;
        }
    }
}
