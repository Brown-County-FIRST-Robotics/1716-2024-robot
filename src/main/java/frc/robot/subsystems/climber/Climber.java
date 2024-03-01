package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  ClimberIO climberIO;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  boolean[] betweenSensors =
      new boolean[] {
        false, false, false, false
      }; // leftBottomSensor, leftTopSensor, rightBottomSensor, rightTopSensor

  public Climber(ClimberIO io) {
    climberIO = io;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/Inputs", inputs);
    checkSensors();
  }

  /**
   * Sets the percent output of the left and right motors Note: this will not go past the sensors
   *
   * @param left the percent to set the left motor to
   * @param right the percent to set the right motor to
   */
  public void setMotors(double left, double right) {
    if (left < 0 && (getSensors()[0][0] || getSensors()[0][1])) {
      left = 0;
    } else if (left > 0 && (getSensors()[1][0] || getSensors()[1][1])) {
      left = 0;
    }
    if (right < 0 && (getSensors()[2][0] || getSensors()[2][1])) {
      right = 0;
    } else if (right > 0 && (getSensors()[3][0] || getSensors()[3][1])) {
      right = 0;
    }
    climberIO.setMotors(clamp(left, -1.0, 1.0), clamp(right, -1.0, 1.0));
  }

  /**
   * Get the values of the magnet sensors and whether the magnet is in the middle of the two
   * sensors.
   *
   * @return an array containing the sensor values and whether the magnet is in the middle, it goes
   *     bottom left, top left, bottom right, top right
   */
  public boolean[][] getSensors() {
    return new boolean[][] {
      {inputs.leftBottomSensor, betweenSensors[0]},
      {inputs.leftTopSensor, betweenSensors[1]},
      {inputs.rightBottomSensor, betweenSensors[2]},
      {inputs.rightTopSensor, betweenSensors[3]}
    };
  }

  //NOTE: This might be flawed...
  private void checkSensors() {
    // if (inputs.leftBottomSensor) {
    //   betweenSensors[0] = !betweenSensors[0];
    // }
    // if (inputs.leftTopSensor) {
    //   betweenSensors[1] = !betweenSensors[1];
    // }
    // if (inputs.rightBottomSensor) {
    //   betweenSensors[2] = !betweenSensors[2];
    // }
    // if (inputs.rightTopSensor) {
    //   betweenSensors[3] = !betweenSensors[3];
    // }
  }

  private double clamp(double value, double min, double max) {
    if (value > max) {
      value = max;
    } else if (value < min) {
      value = min;
    }
    return value;
  }
}
