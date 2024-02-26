package frc.robot.subsystems.climber;

import java.lang.Math.clamp;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  ClimberIO climberIO;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  boolean[] betweenSensors = boolean[4] {false, false, false, false}; //leftBottomSensor, leftTopSensor, rightBottomSensor, rightTopSensor
  boolean doHoldPosition = true;
  double heldPosition = 0.0;

  LoggedTunableNumber holdPostionP = new LoggedTunableNumber("Climber/Hold Position P", 0.1);

  public Climber(ClimberIO io) {
    climberIO = io;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/Inputs", inputs);
    checkSensors();
    checkResetMotorPositions();
    holdPosition();
  }

  /**
   * Sets the voltages of the left and right motors
   * Note: this will not go past the sensors
   *
   * @param leftVoltage the voltage to set the left motor
   * @param rightVoltage the voltage to set the right motor
   */
  public void setVoltage(double leftVoltage, double rightVoltage) {
    if (leftVoltage < 0 && (getSensors()[0][0] || getSensors()[0][1])) {
      leftVoltage = 0;
    }
    else if (leftVoltage > 0 && (getSensors()[1][0] || getSensors()[1][1])) {
      leftVoltage = 0;
    }
    if (rightVoltage < 0 && (getSensors()[2][0] || getSensors()[2][1])) {
      rightVoltage = 0;
    }
    else if (rightVoltage > 0 && (getSensors()[3][0] || getSensors()[3][1])) {
      rightVoltage = 0;
    }
    climberIO.setVoltage(Math.clamp(leftVoltage, -12.0, 12.0), Math.clamp(rightVoltage, -12.0, 12.0));
      //clamp was added in java 21, so it might not be available

    if (leftVoltage == 0 && rightVoltage == 0) {
      doHoldPosition = true;
      heldPosition = climberIO.leftPosition;
    }
    else {
      doHoldPosition = false;
    }
  }

  /**
   * Get the values of the magnet sensors and whether the magnet is in the middle of the two sensors.
   *
   * @return an array containing the sensor values and whether the magnet is in the middle, it goes bottom left, top left, bottom right, top right
   */
  public boolean[] getSensors() {
    return new boolean[][] { {inputs.leftBottomSensor, betweenSensors[0]}, 
      {inputs.leftTopSensor, betweenSensors[1]}, 
      {inputs.rightBottomSensor, betweenSensors[2]}, 
      {inputs.rightTopSensor, betweenSensors[3]} };
  }

  //the same as setVoltage but does not affect doHoldPosition. Used to actually hold the position.
  private void setVoltageInternal(double leftVoltage, double rightVoltage) {
    if (leftVoltage < 0 && (getSensors()[0][0] || getSensors()[0][1])) {
      leftVoltage = 0;
    }
    else if (leftVoltage > 0 && (getSensors()[1][0] || getSensors()[1][1])) {
      leftVoltage = 0;
    }
    if (rightVoltage < 0 && (getSensors()[2][0] || getSensors()[2][1])) {
      rightVoltage = 0;
    }
    else if (rightVoltage > 0 && (getSensors()[3][0] || getSensors()[3][1])) {
      rightVoltage = 0;
    }
    climberIO.setVoltage(Math.clamp(leftVoltage, -12.0, 12.0), Math.clamp(rightVoltage, -12.0, 12.0));
      //clamp was added in java 21, so it might not be available
  }

  private void checkResetMotorPositions() {
    //NOTE: This isn't going to be very accurate, since it resets at the top, bottom, and middle of the sensors (should be good enough though)
    if (getSensors()[0][0] || getSensors()[0][1]) {
      climberIO.leftMotor.setEncoderPosition(0);
    }
    //TODO: FIND POINTS FOR THE TOP
    // else if (getSensors()[1][0] || getSensors()[1][1]) {
    //   climberIO.leftMotor.setEncoderPosition(0);
    // }
    if (getSensors()[2][0] || getSensors()[2][1]) {
      climberIO.rightMotor.setEncoderPosition(0);
    }
    // else if (getSensors()[3][0] || getSensors()[3][1]) {
    //   climberIO.rightMotor.setEncoderPosition(0);
    // }
  }

  private void checkSensors() {
    if climberIO.leftBottomSensor {
      betweenSensors[0] = !betweenSensors[0];
    }
    if climberIO.leftTopSensor {
      betweenSensors[1] = !betweenSensors[1];
    }
    if climberIO.rightBottomSensor {
      betweenSensors[2] = !betweenSensors[2];
    }
    if climberIO.rightTopSensor {
      betweenSensors[3] = !betweenSensors[3];
    }
  }

  private void holdPosition() {
    //TO TEST
    if (doHoldPosition) {
      setVoltageInternal((heldPosition - climberIO.leftPosition) * holdPostionP,
        (heldPosition - climberIO.rightPosition) * holdPostionP);
    }
  }
}
