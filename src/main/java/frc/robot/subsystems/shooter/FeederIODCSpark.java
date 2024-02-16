package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;

public class FeederIODCSpark implements FeederIO {
  CANSparkMax motor;
  RelativeEncoder encoder;
  SparkPIDController pid;
  DigitalInput photoelectricSensor = new DigitalInput(0); // light-based proximity sensor from SICK

  // TEMP CODE
  LoggedTunableNumber feederP = new LoggedTunableNumber("Feeder P", 0);
  LoggedTunableNumber feederI = new LoggedTunableNumber("Feeder I", 0);
  LoggedTunableNumber feederD = new LoggedTunableNumber("Feeder D", 0);
  LoggedTunableNumber feederKV = new LoggedTunableNumber("Feeder KV", 1.0 / 11000.0);
  // END TEMP CODE

  public FeederIODCSpark(int motorId) {
    motor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(Constants.CurrentLimits.NEO550);
    motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    motor.setInverted(true);

    encoder = motor.getEncoder();

    pid = motor.getPIDController();
    pid.setFeedbackDevice(encoder);

    // TEMP CODE
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(11000, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(11000 / 0.5, 0);
    pid.setSmartMotionAllowedClosedLoopError(10, 0);

    // END TEMP CODE

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
    inputs.beamBroken = photoelectricSensor.get(); // TODO: ENSURE THIS DOESN'T NEED INVERTING
  }

  @Override
  public void setVel(double vel) {
    pid.setReference(vel, CANSparkMax.ControlType.kVelocity);
  }
}
