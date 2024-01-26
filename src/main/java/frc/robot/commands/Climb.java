package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class Climb extends Command {
    Climber climber;

    public Climb(Climber climber, DoubleSupplier roll) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        
    }
}
