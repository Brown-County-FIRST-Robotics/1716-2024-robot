package frc.robot.subsystems.shooter;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.Constants;

public class FeederIODCSpark implements FeederIO {
  CANSparkMax motor;
  SparkAnalogSensor encoder;
  SparkPIDController pid;
  DigitalInput photoelectricSensor = new DigitalInput(0); //light-based proximity sensor from SICK

  // TEMP CODE
  LoggedTunableNumber feederP = new LoggedTunableNumber("Feeder P", 0);
  LoggedTunableNumber feederI = new LoggedTunableNumber("Feeder I", 0);
  LoggedTunableNumber feederD = new LoggedTunableNumber("Feeder D", 0);
  LoggedTunableNumber feederKV = new LoggedTunableNumber("Feeder KV", 1.0 / 300.0);
  // END TEMP CODE

  public FeederIODCSpark(int motorId) {
    motor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushed);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(Constants.CurrentLimits.GENERIC_BRUSHED);
    motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    motor.setInverted(true);

    encoder = motor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
    encoder.setPositionConversionFactor(1 / 3.3);

    pid = motor.getPIDController();
    pid.setFeedbackDevice(encoder);

    // TEMP CODE
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(300, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(1200, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.01, 0);
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMaxInput(1);
    pid.setPositionPIDWrappingMinInput(0);
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
    inputs.beamBroken = photoelectricSensor.get(); //TODO: ENSURE THIS DOESN'T NEED INVERTING
  }

  @Override
  public void setPosition(double position) {
    pid.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }
}
