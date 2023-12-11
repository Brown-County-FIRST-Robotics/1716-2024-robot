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
    reconfigure();
    fl.burnFlash();
    fr.burnFlash();
    bl.burnFlash();
    br.burnFlash();
    Logger.getInstance().recordMetadata("FLFW", fl.getFirmwareString());
    Logger.getInstance().recordMetadata("FRFW", fr.getFirmwareString());
    Logger.getInstance().recordMetadata("BLFW", bl.getFirmwareString());
    Logger.getInstance().recordMetadata("BRFW", br.getFirmwareString());
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
  public void reconfigure() {

    if (ffTuner.hasChanged()) {
      flPID.setFF(ffTuner.get());
      frPID.setFF(ffTuner.get());
      blPID.setFF(ffTuner.get());
      brPID.setFF(ffTuner.get());
    }
    if (pTuner.hasChanged()) {
      flPID.setP(pTuner.get());
      frPID.setP(pTuner.get());
      blPID.setP(pTuner.get());
      brPID.setP(pTuner.get());
    }
    if (iTuner.hasChanged()) {
      flPID.setI(iTuner.get());
      frPID.setI(iTuner.get());
      blPID.setI(iTuner.get());
      brPID.setI(iTuner.get());
    }
    if (dTuner.hasChanged()) {
      flPID.setD(dTuner.get());
      frPID.setD(dTuner.get());
      blPID.setD(dTuner.get());
      brPID.setD(dTuner.get());
    }
  }

  @Override
  public void updateInputs(MecanumIOInputs inputs) {
    inputs.flTemp = fl.getMotorTemperature();
    inputs.frTemp = fr.getMotorTemperature();
    inputs.blTemp = bl.getMotorTemperature();
    inputs.brTemp = br.getMotorTemperature();
    inputs.flPos = flEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
    inputs.frPos = frEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
    inputs.blPos = blEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
    inputs.brPos = brEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER;
  }
}
