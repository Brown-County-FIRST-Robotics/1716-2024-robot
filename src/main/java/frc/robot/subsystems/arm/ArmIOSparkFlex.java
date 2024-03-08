package frc.robot.subsystems.arm;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.LoggedTunableNumber;
import org.xml.sax.helpers.AttributesImpl;

public class ArmIOSparkFlex implements ArmIO {
  CANSparkMax controller;
  SparkPIDController pid;
  AbsoluteEncoder absencoder;
  private static final double GEAR_RATIO = 25.0 * 72.0 / 15.0;
  private static final double FREE_RPM = 5676.0;
  LoggedTunableNumber ffTuner = new LoggedTunableNumber("Arm/ff_tuner", GEAR_RATIO / FREE_RPM);
  LoggedTunableNumber pTuner = new LoggedTunableNumber("Arm/p_tuner", 0 * GEAR_RATIO / FREE_RPM);
  LoggedTunableNumber iTuner = new LoggedTunableNumber("Arm/i_tuner", 0.0);
  LoggedTunableNumber dTuner = new LoggedTunableNumber("Arm/d_tuner", 0.0);
  LoggedTunableNumber offset = new LoggedTunableNumber("Arm/offset", 0.496);
  RelativeEncoder relencoder;
  Timer nom=new Timer();
  Rotation2d relEncoderZero=new Rotation2d();

  public ArmIOSparkFlex(int id) {
    controller = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    pid = controller.getPIDController();
    absencoder = controller.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    relencoder=controller.getEncoder();
    controller.restoreFactoryDefaults();
    relencoder.setPositionConversionFactor(1.0/GEAR_RATIO);
    absencoder.setInverted(true);
    controller.setInverted(true);

    controller.setIdleMode(CANSparkBase.IdleMode.kBrake);
    controller.setSmartCurrentLimit(Constants.CurrentLimits.NEO);
    pid.setFeedbackDevice(relencoder);
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(FREE_RPM / GEAR_RATIO, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(FREE_RPM / GEAR_RATIO, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.004, 0);
    ffTuner.attach(pid::setFF);
    pTuner.attach(pid::setP);
    iTuner.attach(pid::setI);
    dTuner.attach(pid::setD);
    controller.burnFlash();
    CustomAlerts.makeOverTempAlert(controller, 60, 50, "Arm motor");
    rezero();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = getProcessedRel();
    inputs.omega = relencoder.getVelocity();
    inputs.appliedOutput = controller.getAppliedOutput();
    inputs.temperature = controller.getMotorTemperature();
    inputs.current=controller.getOutputCurrent();
    if(relencoder.getVelocity()>0.1||absencoder.getVelocity()>0.1){
      nom.restart();
    }
    if(nom.hasElapsed(0.5)){
      rezero();
    }
  }
  private Rotation2d getAbs(){
    return Rotation2d.fromRotations(absencoder.getPosition() - offset.get());
  }
  private Rotation2d getUnprocessedRel(){
    return Rotation2d.fromRotations(relencoder.getPosition());
  }
  private Rotation2d getProcessedRel(){
    return relEncoderZero.plus(getUnprocessedRel());
  }
  private void rezero(){
    relEncoderZero=getAbs().minus(getUnprocessedRel());
    nom.restart();
  }

  @Override
  public void setAngle(Rotation2d cmdAng, double arbFF) {
    double adjustedRelCmd =relencoder.getPosition()
            - (relencoder.getPosition() % 1.0)
            + cmdAng.minus(relEncoderZero).getRotations();
    if (Math.abs(adjustedRelCmd - relencoder.getPosition())
        > Math.abs(1.0 + adjustedRelCmd - relencoder.getPosition())) {
      adjustedRelCmd += 1.0;
    }
    if (Math.abs(adjustedRelCmd - relencoder.getPosition())
        > Math.abs(-1.0 + adjustedRelCmd - relencoder.getPosition())) {
      adjustedRelCmd -= 1.0;
    }
    pid.setReference(
         adjustedRelCmd,
        CANSparkBase.ControlType.kSmartMotion,
        0,
        arbFF,
        SparkPIDController.ArbFFUnits.kVoltage);
  }
}
