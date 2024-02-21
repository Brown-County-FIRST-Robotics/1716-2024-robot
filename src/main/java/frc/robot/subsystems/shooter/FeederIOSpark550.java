package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;

public class FeederIOSpark550 implements FeederIO {
  CANSparkMax motor;
  RelativeEncoder encoder;
  SparkPIDController pid;
  DigitalInput openContact;
  DigitalInput closedContact;

  LoggedTunableNumber feederP = new LoggedTunableNumber("Feeder P", 0);
  LoggedTunableNumber feederI = new LoggedTunableNumber("Feeder I", 0);
  LoggedTunableNumber feederD = new LoggedTunableNumber("Feeder D", 0);
  LoggedTunableNumber feederKV = new LoggedTunableNumber("Feeder KV", 1.0 / 11000.0);

  public FeederIOSpark550(int motorId, int openContactPin, int closedContactPin) {
    openContact = new DigitalInput(openContactPin);
    closedContact = new DigitalInput(closedContactPin);
    motor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(Constants.CurrentLimits.NEO550);
    motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    motor.setInverted(true);

    encoder = motor.getEncoder();

    pid = motor.getPIDController();
    pid.setFeedbackDevice(encoder);

    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(11000, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(11000 / 0.5, 0);
    pid.setSmartMotionAllowedClosedLoopError(10, 0);

    feederKV.attach(pid::setFF);
    feederP.attach(pid::setP);
    feederI.attach(pid::setI);
    feederD.attach(pid::setD);

    motor.burnFlash();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();
    inputs.current = motor.getOutputCurrent();
    inputs.closedContact = closedContact.get();
    inputs.openContact = openContact.get();
  }

  @Override
  public void setVel(double vel) {
    pid.setReference(vel, CANSparkMax.ControlType.kVelocity);
  }
}
