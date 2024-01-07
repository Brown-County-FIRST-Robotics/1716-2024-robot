package frc.robot.subsystems.mecanum;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class MecanumIOSpark implements MecanumIO {
  public static final double EFFECTIVE_WHEEL_DIAMETER = 0.05411255411255412;
  CANSparkMax fl;
  CANSparkMax fr;
  CANSparkMax bl;
  CANSparkMax br;
  RelativeEncoder flEncoder;
  RelativeEncoder frEncoder;
  RelativeEncoder blEncoder;
  RelativeEncoder brEncoder;
  SparkMaxPIDController flPID;
  SparkMaxPIDController frPID;
  SparkMaxPIDController blPID;
  SparkMaxPIDController brPID;
  LoggedTunableNumber ffTuner = new LoggedTunableNumber("Mecanum FF", 1.0 / 6500);
  LoggedTunableNumber pTuner = new LoggedTunableNumber("Mecanum P", 0);
  LoggedTunableNumber iTuner = new LoggedTunableNumber("Mecanum I", 0);
  LoggedTunableNumber dTuner = new LoggedTunableNumber("Mecanum D", 0);

  public MecanumIOSpark(int flID, int frID, int blID, int brID) {
    fl = new CANSparkMax(flID, CANSparkMaxLowLevel.MotorType.kBrushless);
    flEncoder = fl.getEncoder();
    flPID = fl.getPIDController();
    fr = new CANSparkMax(frID, CANSparkMaxLowLevel.MotorType.kBrushless);
    frEncoder = fr.getEncoder();
    frPID = fr.getPIDController();
    bl = new CANSparkMax(blID, CANSparkMaxLowLevel.MotorType.kBrushless);
    blEncoder = bl.getEncoder();
    blPID = bl.getPIDController();
    br = new CANSparkMax(brID, CANSparkMaxLowLevel.MotorType.kBrushless);
    brEncoder = br.getEncoder();
    brPID = br.getPIDController();

    fl.setSmartCurrentLimit(30);
    fr.setSmartCurrentLimit(30);
    bl.setSmartCurrentLimit(30);
    br.setSmartCurrentLimit(30);

    flPID.setFeedbackDevice(flEncoder);
    flPID.setOutputRange(-1, 1);
    frPID.setFeedbackDevice(frEncoder);
    frPID.setOutputRange(-1, 1);
    blPID.setFeedbackDevice(blEncoder);
    blPID.setOutputRange(-1, 1);
    brPID.setFeedbackDevice(brEncoder);
    brPID.setOutputRange(-1, 1);

    ffTuner.attach(
        (Double v) -> {
          flPID.setFF(v);
          frPID.setFF(v);
          blPID.setFF(v);
          brPID.setFF(v);
        });
    pTuner.attach(
        (Double v) -> {
          flPID.setP(v);
          frPID.setP(v);
          blPID.setP(v);
          brPID.setP(v);
        });
    iTuner.attach(
        (Double v) -> {
          flPID.setI(v);
          frPID.setI(v);
          blPID.setI(v);
          brPID.setI(v);
        });
    dTuner.attach(
        (Double v) -> {
          flPID.setD(v);
          frPID.setD(v);
          blPID.setD(v);
          brPID.setD(v);
        });

    fl.burnFlash();
    fr.burnFlash();
    bl.burnFlash();
    br.burnFlash();
    Logger.recordMetadata("FLFW", fl.getFirmwareString());
    Logger.recordMetadata("FRFW", fr.getFirmwareString());
    Logger.recordMetadata("BLFW", bl.getFirmwareString());
    Logger.recordMetadata("BRFW", br.getFirmwareString());
  }

  @Override
  public void setSpeeds(MecanumDriveWheelSpeeds cmd) {
    flPID.setReference(
        60 * cmd.frontLeftMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        CANSparkMax.ControlType.kVelocity);
    frPID.setReference(
        60 * cmd.frontRightMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        CANSparkMax.ControlType.kVelocity);
    blPID.setReference(
        60 * cmd.rearLeftMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        CANSparkMax.ControlType.kVelocity);
    brPID.setReference(
        60 * cmd.rearRightMetersPerSecond / EFFECTIVE_WHEEL_DIAMETER,
        CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void updateInputs(MecanumIOInputs inputs) {
    inputs.flTemp = fl.getMotorTemperature();
    inputs.frTemp = fr.getMotorTemperature();
    inputs.blTemp = bl.getMotorTemperature();
    inputs.brTemp = br.getMotorTemperature();
    inputs.flPos = flEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
    inputs.flVel = flEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0;
    inputs.frPos = frEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
    inputs.frVel = frEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0;
    inputs.blPos = blEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
    inputs.blVel = blEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0;
    inputs.brPos = brEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
    inputs.brVel = brEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0;
    inputs.flOut = fl.getAppliedOutput();
    inputs.frOut = fr.getAppliedOutput();
    inputs.blOut = bl.getAppliedOutput();
    inputs.brOut = br.getAppliedOutput();
  }
}
