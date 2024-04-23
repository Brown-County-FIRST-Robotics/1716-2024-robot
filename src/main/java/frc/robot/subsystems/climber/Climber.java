package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  final ClimberIO climberIO;
  final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final double[] sensorOffsetFromBottom = {
    3.5, 6
  }; // The bottom sensors are a certain distance from the hardware limit, this amount
  private final double maxSpeed = 1;
  final boolean[] downLocked = {
    true, true
  }; // doesn't know the current position, so it can't move down (left, right)

  public Climber(ClimberIO io) {
    climberIO = io;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/Inputs", inputs);
    checkBottomSensors();
    if (downLocked[0]
        && (inputs.leftPosition > sensorOffsetFromBottom[0] || inputs.leftTopSensor)) {
      climberIO.setMotorEncoderPosition(false, 99999);
      downLocked[0] = false;
    }
    if (downLocked[1]
        && (inputs.rightPosition > sensorOffsetFromBottom[1] || inputs.rightTopSensor)) {
      climberIO.setMotorEncoderPosition(true, 99999);
      downLocked[1] = false;
    }
  }

  /**
   * Sets the percent output of the left and right motors Note: this will not go past the sensors
   *
   * @param left the percent to set the left motor to
   * @param right the percent to set the right motor to
   */
  public void setMotors(double left, double right) {
    if (left < 0 && (inputs.leftPosition < -sensorOffsetFromBottom[0] || downLocked[0])) {
      left = 0;
    } else if (left > 0 && inputs.leftTopSensor) {
      left = 0;
    }
    if (right < 0 && (inputs.rightPosition < -sensorOffsetFromBottom[1] || downLocked[1])) {
      right = 0;
    } else if (right > 0 && inputs.rightTopSensor) {
      right = 0;
    }
    climberIO.setMotors(clamp(left, -maxSpeed, maxSpeed), clamp(right, -maxSpeed, maxSpeed));
  }

  private void checkBottomSensors() {
    if (inputs.leftBottomSensor) {
      climberIO.setMotorEncoderPosition(false, 0);
    }
    if (inputs.rightBottomSensor) {
      climberIO.setMotorEncoderPosition(true, 0);
    }
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
