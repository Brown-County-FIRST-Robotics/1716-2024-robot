package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOSparkFlex implements ShooterIO {
  CANSparkFlex motor;
  DigitalInput beamBreakSensor;
  RelativeEncoder encoder;

  public ShooterIOSparkFlex(int motorID, int beamBreakID) {
    motor = new CANSparkFlex(motorID, CANSparkLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();
    beamBreakSensor = new DigitalInput(beamBreakID);
    motor.setSmartCurrentLimit(30);
    motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.beamBroke = beamBreakSensor.get();
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity() / 60.0;
    inputs.motorTemperature = motor.getMotorTemperature();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.motorOutput = motor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
