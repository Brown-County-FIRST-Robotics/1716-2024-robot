package frc.robot.subsystems.arm;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.LoggedTunableNumber;

/** IO implementation for Arm using a Spark Flex */
public class ArmIOSparkFlex implements ArmIO {
  final CANSparkMax controller;
  final SparkPIDController pid;
  final AbsoluteEncoder absEncoder;
  private static final double GEAR_RATIO = 25.0 * 72.0 / 15.0;
  private static final double FREE_RPM = 5676.0;
  final LoggedTunableNumber ffTuner =
      new LoggedTunableNumber("Arm/ff_tuner", GEAR_RATIO / FREE_RPM);
  final LoggedTunableNumber pTuner =
      new LoggedTunableNumber("Arm/p_tuner", 1.0 * GEAR_RATIO / FREE_RPM);
  final LoggedTunableNumber iTuner = new LoggedTunableNumber("Arm/i_tuner", 0.0);
  final LoggedTunableNumber dTuner = new LoggedTunableNumber("Arm/d_tuner", 0.0);
  final LoggedTunableNumber offset = new LoggedTunableNumber("Arm/offset", 0.496);

  /**
   * Creates a <code>ArmIOSparkFlex</code> from a CAN id
   *
   * @param id The CAN id of the arm
   */
  public ArmIOSparkFlex(int id) {
    controller = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    pid = controller.getPIDController();
    absEncoder = controller.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    controller.restoreFactoryDefaults();
    absEncoder.setInverted(true);
    controller.setInverted(true);

    controller.setIdleMode(CANSparkBase.IdleMode.kBrake);
    controller.setSmartCurrentLimit(Constants.CurrentLimits.NEO);
    pid.setFeedbackDevice(absEncoder);
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(FREE_RPM / GEAR_RATIO, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(0.5 * FREE_RPM / GEAR_RATIO, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.004, 0);
    ffTuner.attach(pid::setFF);
    pTuner.attach(pid::setP);
    iTuner.attach(pid::setI);
    dTuner.attach(pid::setD);

    controller.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 20);
    controller.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 20);

    controller.burnFlash();
    CustomAlerts.makeOverTempAlert(controller, 60, 50, "Arm motor");
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle =
        Rotation2d.fromRotations(absEncoder.getPosition())
            .minus(Rotation2d.fromRotations(offset.get()));
    inputs.omega = absEncoder.getVelocity();
    inputs.appliedOutput = controller.getAppliedOutput();
    inputs.temperature = controller.getMotorTemperature();
    inputs.current = controller.getOutputCurrent();
  }

  @Override
  public void setAngle(Rotation2d cmdAng, double arbFF) {
    double adjustedRelCmd =
        absEncoder.getPosition()
            - (absEncoder.getPosition() % 1.0)
            + cmdAng.plus(Rotation2d.fromRotations(offset.get())).getRotations();
    if (Math.abs(adjustedRelCmd - absEncoder.getPosition())
        > Math.abs(1.0 + adjustedRelCmd - absEncoder.getPosition())) {
      adjustedRelCmd += 1.0;
    }
    if (Math.abs(adjustedRelCmd - absEncoder.getPosition())
        > Math.abs(-1.0 + adjustedRelCmd - absEncoder.getPosition())) {
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
