package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    ClimberIO io;
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/Inputs", inputs);
    }

    public void setVoltage(double leftVoltage, double rightVoltage) {
        io.setVoltage(leftVoltage, rightVoltage);
    }
}
