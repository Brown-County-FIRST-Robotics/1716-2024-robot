package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.LoggedTunableNumber;

public class ShooterIOSparkFlexes implements ShooterIO {
  private final SparkPIDController pid1;
  private final SparkPIDController pid2;

  CANSparkFlex motor1;
  CANSparkFlex motor2;
  DigitalInput beamBreakSensor;
  RelativeEncoder encoder1;
  RelativeEncoder encoder2;
  LoggedTunableNumber ffTuner = new LoggedTunableNumber("Mecanum FF", 1.0 / 6500);
  LoggedTunableNumber pTuner = new LoggedTunableNumber("Mecanum P", 0);
  LoggedTunableNumber iTuner = new LoggedTunableNumber("Mecanum I", 0);
  LoggedTunableNumber dTuner = new LoggedTunableNumber("Mecanum D", 0);

  public ShooterIOSparkFlexes(int motorID1, int motorID2, int beamBreakID) {
    motor1 = new CANSparkFlex(motorID1, CANSparkLowLevel.MotorType.kBrushless);
    motor2 = new CANSparkFlex(motorID2, CANSparkLowLevel.MotorType.kBrushless);
    encoder1 = motor1.getEncoder();
    encoder2 = motor2.getEncoder();
    motor1.setSmartCurrentLimit(80);
    motor1.setIdleMode(CANSparkBase.IdleMode.kCoast);
    motor2.setSmartCurrentLimit(80);
    motor2.setIdleMode(CANSparkBase.IdleMode.kCoast);
    pid1 = motor1.getPIDController();
    pid2 = motor2.getPIDController();

    pid1.setFeedbackDevice(encoder1);
    pid1.setOutputRange(-1, 1, 0);
    pid2.setFeedbackDevice(encoder2);
    pid2.setOutputRange(-1, 1, 0);

    ffTuner.attach(
        (Double v) -> {
          pid1.setFF(v, 0);
          pid1.setFF(v, 1);
          pid2.setFF(v, 0);
          pid2.setFF(v, 1);
        });
    pTuner.attach(
        (Double v) -> {
          pid1.setP(v, 0);
          pid1.setP(v, 1);
          pid2.setP(v, 0);
          pid2.setP(v, 1);
        });
    iTuner.attach(
        (Double v) -> {
          pid1.setI(v, 0);
          pid1.setI(v, 1);
          pid2.setI(v, 0);
          pid2.setI(v, 1);
        });
    dTuner.attach(
        (Double v) -> {
          pid1.setD(v, 0);
          pid1.setD(v, 1);
          pid2.setD(v, 0);
          pid2.setD(v, 1);
        });
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.position[0] = encoder1.getPosition();
    inputs.velocity[0] = encoder1.getVelocity();
    inputs.motorTemperature[0] = motor1.getMotorTemperature();
    inputs.motorCurrent[0] = motor1.getOutputCurrent();
    inputs.motorOutput[0] = motor1.getAppliedOutput();

    inputs.position[1] = encoder2.getPosition();
    inputs.velocity[1] = encoder2.getVelocity();
    inputs.motorTemperature[1] = motor2.getMotorTemperature();
    inputs.motorCurrent[1] = motor2.getOutputCurrent();
    inputs.motorOutput[1] = motor2.getAppliedOutput();
  }

  @Override
  public void setVoltage(double voltage) {
    motor1.setVoltage(voltage);
  }

  @Override
  public void setVelocity(double vel) {
    if (vel != 0) {
      pid1.setReference(-vel, CANSparkBase.ControlType.kVelocity, 0);
      pid2.setReference(vel, CANSparkBase.ControlType.kVelocity, 0);
    } else {
      motor1.set(0);
      motor2.set(0);
    }
  }

  @Override
  public void smallStepTo(double pos1, double pos2) {
    pid1.setReference(pos1, CANSparkBase.ControlType.kSmartMotion, 1);
    pid2.setReference(pos2, CANSparkBase.ControlType.kSmartMotion, 1);
  }
}
