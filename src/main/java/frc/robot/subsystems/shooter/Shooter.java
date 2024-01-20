package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  ShooterIO io;

  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Inputs", inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Inputs", inputs);
  }
  public void cmdvel(double voltage) {
    io.setVelocity(voltage);
  }

  public void cmdVoltage(double voltage) {
    io.setVoltage(voltage);
  }
}
