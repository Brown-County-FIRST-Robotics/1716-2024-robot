package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;

/** IO implementation of the feeder for a NEO 550 */
public class FeederIOSpark550 implements FeederIO {
  final CANSparkMax motor;
  final RelativeEncoder encoder;
  final SparkPIDController pid;
  final DigitalInput openContact;
  final DigitalInput closedContact;

  final LoggedTunableNumber feederP = new LoggedTunableNumber("Feeder P", 0);
  final LoggedTunableNumber feederI = new LoggedTunableNumber("Feeder I", 0);
  final LoggedTunableNumber feederD = new LoggedTunableNumber("Feeder D", 0);
  final LoggedTunableNumber feederKV = new LoggedTunableNumber("Feeder KV", 1.0 / 11000.0);

  /**
   * Constructs the IO from a CAN ID and pin IDs
   *
   * @param motorId The CAN ID of the feeder motor
   * @param openContactPin The digital input pin for the open contact
   * @param closedContactPin The digital input pin for the closed contact
   */
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
    inputs.temperature = motor.getMotorTemperature();
    inputs.appliedOutput = motor.getAppliedOutput();
  }

  @Override
  public void setVel(double vel) {
    pid.setReference(vel, CANSparkMax.ControlType.kVelocity);
  }
}
