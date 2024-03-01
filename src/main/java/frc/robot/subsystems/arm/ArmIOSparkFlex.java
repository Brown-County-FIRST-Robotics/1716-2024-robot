package frc.robot.subsystems.arm;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.LoggedTunableNumber;

public class ArmIOSparkFlex implements ArmIO {
  CANSparkMax controller;
  SparkPIDController pid;
  AbsoluteEncoder encoder;
  private static final double GEAR_RATIO = 25.0 * 72.0 / 15.0;
  private static final double FREE_RPM = 5676.0;
  LoggedTunableNumber ffTuner = new LoggedTunableNumber("Arm/ff_tuner", GEAR_RATIO / FREE_RPM);
  LoggedTunableNumber pTuner = new LoggedTunableNumber("Arm/p_tuner", 0.5 * GEAR_RATIO / FREE_RPM);
  LoggedTunableNumber iTuner = new LoggedTunableNumber("Arm/i_tuner", 0.0);
  LoggedTunableNumber dTuner = new LoggedTunableNumber("Arm/d_tuner", 0.0);
  LoggedTunableNumber offset = new LoggedTunableNumber("Arm/offset", 0.496);

  public ArmIOSparkFlex(int id) {
    controller = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    pid = controller.getPIDController();
    encoder = controller.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    controller.restoreFactoryDefaults();
    encoder.setInverted(true);
    controller.setInverted(true);

    controller.setIdleMode(CANSparkBase.IdleMode.kBrake);
    controller.setSmartCurrentLimit(Constants.CurrentLimits.NEO);
    pid.setFeedbackDevice(encoder);
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(FREE_RPM / GEAR_RATIO, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(15, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.004, 0);
    ffTuner.attach(pid::setFF);
    pTuner.attach(pid::setP);
    iTuner.attach(pid::setI);
    dTuner.attach(pid::setD);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5,20);
    // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4,1000); // CHANGE THIS WHEN NOT USING OLI CABLE
    // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3,1000);
    // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2,1000);
    // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0,1000/20);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6,20);
    // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1,1000/50);
    controller.setCANTimeout(0);

    controller.burnFlash();
    CustomAlerts.makeOverTempAlert(controller, 60, 50, "Arm motor");
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(encoder.getPosition() - offset.get());
    inputs.omega = encoder.getVelocity() / 60.0;
    inputs.appliedOutput = controller.getAppliedOutput();
    inputs.temperature = controller.getMotorTemperature();
  }

  @Override
  public void setAngle(Rotation2d cmdAng, double arbFF) {
    pid.setReference(
        cmdAng.getRotations() + offset.get(),
        CANSparkBase.ControlType.kSmartMotion,
        0,
        arbFF,
        SparkPIDController.ArbFFUnits.kVoltage);
  }
}
