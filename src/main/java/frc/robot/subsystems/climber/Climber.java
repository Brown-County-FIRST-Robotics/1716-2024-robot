package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  ClimberIO climberIO;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    climberIO = io;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/Inputs", inputs);
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {
    climberIO.setVoltage(leftVoltage, rightVoltage);
  }

  /**
   * Get the values of the magnet sensors.
   *
   * @return an array containing the sensor values, it goes bottom left, top left, bottom right, top right
   */
  public boolean[] getSensors() {
    return new boolean[] {inputs.leftBottomSensor, inputs.leftTopSensor, inputs.rightBottomSensor, inputs.rightTopSensor};
  }
}
