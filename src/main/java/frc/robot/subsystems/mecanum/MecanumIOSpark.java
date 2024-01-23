package frc.robot.subsystems.mecanum;

import com.revrobotics.*;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** The mecanum IO implementation for 4 SPARKMAX motor controllers */
public class MecanumIOSpark implements MecanumIO {
  static final double EFFECTIVE_WHEEL_DIAMETER = 0.05411255411255412;
  CANSparkMax fl;
  CANSparkMax fr;
  CANSparkMax bl;
  CANSparkMax br;
  RelativeEncoder flEncoder;
  RelativeEncoder frEncoder;
  RelativeEncoder blEncoder;
  RelativeEncoder brEncoder;
  SparkPIDController flPID;
  SparkPIDController frPID;
  SparkPIDController blPID;
  SparkPIDController brPID;
  LoggedTunableNumber ffTuner = new LoggedTunableNumber("Mecanum FF", 1.0 / 6500);
  LoggedTunableNumber pTuner = new LoggedTunableNumber("Mecanum P", 0);
  LoggedTunableNumber iTuner = new LoggedTunableNumber("Mecanum I", 0);
  LoggedTunableNumber dTuner = new LoggedTunableNumber("Mecanum D", 0);

  /**
   * Constructs a <code>MecanumIOSpark</code> from CAN IDs
   *
   * @param flID Front left CAN ID
   * @param frID Front right CAN ID
   * @param blID Back left CAN ID
   * @param brID Back right CAN ID
   */
  public MecanumIOSpark(int flID, int frID, int blID, int brID) {
    fl = new CANSparkMax(flID, CANSparkLowLevel.MotorType.kBrushless);
    flEncoder = fl.getEncoder();
    flPID = fl.getPIDController();
    fr = new CANSparkMax(frID, CANSparkLowLevel.MotorType.kBrushless);
    frEncoder = fr.getEncoder();
    frPID = fr.getPIDController();
    bl = new CANSparkMax(blID, CANSparkLowLevel.MotorType.kBrushless);
    blEncoder = bl.getEncoder();
    blPID = bl.getPIDController();
    br = new CANSparkMax(brID, CANSparkLowLevel.MotorType.kBrushless);
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
    CustomAlerts.makeOverTempAlert(fl, 60, 50, "FL motor");
    CustomAlerts.makeOverTempAlert(fr, 60, 50, "FR motor");
    CustomAlerts.makeOverTempAlert(bl, 60, 50, "BL motor");
    CustomAlerts.makeOverTempAlert(br, 60, 50, "BR motor");
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
    inputs.pos =
        new MecanumDriveWheelPositions(
            flEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER,
            frEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER,
            blEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER,
            brEncoder.getPosition() * EFFECTIVE_WHEEL_DIAMETER);
    inputs.vel =
        new MecanumDriveWheelSpeeds(
            flEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0,
            frEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0,
            blEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0,
            brEncoder.getVelocity() * EFFECTIVE_WHEEL_DIAMETER / 60.0);
    inputs.flOut = fl.getAppliedOutput();
    inputs.frOut = fr.getAppliedOutput();
    inputs.blOut = bl.getAppliedOutput();
    inputs.brOut = br.getAppliedOutput();
  }
}
