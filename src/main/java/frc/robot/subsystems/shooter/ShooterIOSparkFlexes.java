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


  public ShooterIOSparkFlexes(int motorID1,int motorID2, int beamBreakID) {
    motor1 = new CANSparkFlex(motorID1, CANSparkLowLevel.MotorType.kBrushless);
    motor2 = new CANSparkFlex(motorID2, CANSparkLowLevel.MotorType.kBrushless);
    encoder1 = motor1.getEncoder();
    encoder2 = motor2.getEncoder();
    beamBreakSensor = new DigitalInput(beamBreakID);
    motor1.setSmartCurrentLimit(80);
    motor1.setIdleMode(CANSparkBase.IdleMode.kCoast);
    motor2.setSmartCurrentLimit(80);
    motor2.setIdleMode(CANSparkBase.IdleMode.kCoast);
    pid1=motor1.getPIDController();
    pid2=motor2.getPIDController();

    pid1.setFeedbackDevice(encoder1);
    pid1.setOutputRange(-1, 1);
    pid2.setFeedbackDevice(encoder2);
    pid2.setOutputRange(-1, 1);


    ffTuner.attach(
        (Double v) -> {
          pid1.setFF(v);
          pid2.setFF(v);
        });
    pTuner.attach(
        (Double v) -> {
          pid1.setP(v);
          pid2.setP(v);
        });
    iTuner.attach(
        (Double v) -> {
          pid1.setI(v);
          pid2.setI(v);
        });
    dTuner.attach(
        (Double v) -> {
          pid1.setD(v);
          pid2.setD(v);
        });


  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.beamBroke = beamBreakSensor.get();
    inputs.position[0] = encoder1.getPosition();
    inputs.velocity[0] = encoder1.getVelocity() / 60.0;
    inputs.motorTemperature[0] = motor1.getMotorTemperature();
    inputs.motorCurrent[0] = motor1.getOutputCurrent();
    inputs.motorOutput[0] = motor1.getAppliedOutput();


    inputs.position[1] = encoder2.getPosition();
    inputs.velocity[1] = encoder2.getVelocity() / 60.0;
    inputs.motorTemperature[1] = motor2.getMotorTemperature();
    inputs.motorCurrent[1] = motor2.getOutputCurrent();
    inputs.motorOutput[1] = motor2.getAppliedOutput();
  }

  @Override
  public void setVoltage(double voltage) {
    motor1.setVoltage(voltage);
  }
  @Override
  public void setVelocity(double vel){
    if(vel!=0){
    pid1.setReference(vel, CANSparkBase.ControlType.kVelocity);
    pid2.setReference(-vel, CANSparkBase.ControlType.kVelocity);
    }else{
      motor1.set(0);
      motor2.set(0);
    }

  }
}
