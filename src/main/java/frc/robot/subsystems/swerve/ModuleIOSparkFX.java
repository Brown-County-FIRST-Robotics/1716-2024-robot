package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** IO layer for a SDS MK4i L2 swerve module using a Falcon 500 as thrust, and a Neo as steering */
public class ModuleIOSparkFX implements ModuleIO {
  private final double THRUST_DISTANCE_PER_TICK = .0254 * 4.0 * Math.PI / 6.75;
  private final CANSparkMax steer;
  private final SparkAnalogSensor encoder;
  private final SparkPIDController pid;
  private final TalonFX thrust;
  StatusSignal<Double> velSignal;
  StatusSignal<Double> posSignal;
  StatusSignal<Double> errSignal;
  StatusSignal<Double> tempSignal;
  double offset;

  String name;
  LoggedTunableNumber thrustP = new LoggedTunableNumber("Thrust P", 0);
  LoggedTunableNumber thrustI = new LoggedTunableNumber("Thrust I", 0);
  LoggedTunableNumber thrustD = new LoggedTunableNumber("Thrust D", 0);
  LoggedTunableNumber thrustKV = new LoggedTunableNumber("Thrust KV", 60.0 / 6380.0);
  LoggedTunableNumber steerP = new LoggedTunableNumber("Steer P", 0);
  LoggedTunableNumber steerI = new LoggedTunableNumber("Steer I", 0);
  LoggedTunableNumber steerD = new LoggedTunableNumber("Steer D", 0);
  LoggedTunableNumber steerKV = new LoggedTunableNumber("Steer KV", 1.0 / 300.0);
  LoggedTunableNumber offsetTun;
  double off;

  /**
   * Makes a new instance using CAN IDs
   *
   * @param thrustID Thrust motor CAN ID
   * @param steerID Steer motor controller CAN ID
   * @param name The name of the module (eg. "FL", "BR")
   */
  public ModuleIOSparkFX(int thrustID, int steerID, String name) {
    this.name = name;
    thrust = new TalonFX(thrustID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Audio.BeepOnConfig = false;
    config.Audio.BeepOnBoot = false;
    config.Audio.AllowMusicDurDisable = true;
    config.Slot0.kV = thrustKV.get();
    thrust.getConfigurator().refresh(config.CustomParams);
    offsetTun = new LoggedTunableNumber(name + "_offset");
    if (steerID == 10) {
      off=0.824;
    } else if (steerID == 11) {
      off=0;
    } else if (steerID == 12) {
      off=0;
    } else if (steerID == 13) {
      off=0.045;
    }
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    thrust.getConfigurator().apply(config);
    velSignal = thrust.getRotorVelocity();
    posSignal = thrust.getRotorPosition();
    errSignal = thrust.getClosedLoopError();
    tempSignal = thrust.getDeviceTemp();
    velSignal.setUpdateFrequency(50.0);
    posSignal.setUpdateFrequency(50.0);
    errSignal.setUpdateFrequency(50.0);
    tempSignal.setUpdateFrequency(20.0);
    thrust.optimizeBusUtilization();
    steer = new CANSparkMax(steerID, CANSparkLowLevel.MotorType.kBrushless);
    steer.restoreFactoryDefaults();
    pid = steer.getPIDController();
    encoder = steer.getAnalog(SparkAnalogSensor.Mode.kAbsolute);

    encoder.setPositionConversionFactor(1 / 3.3);
    encoder.setInverted(true);
    pid.setFeedbackDevice(encoder);

    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(300, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(1200, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.01, 0);
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMaxInput(1);
    pid.setPositionPIDWrappingMinInput(0);
    steer.setSmartCurrentLimit(Constants.CurrentLimits.NEO);

    steerKV.attach(pid::setFF);
    steerP.attach(pid::setP);
    steerI.attach(pid::setI);
    steerD.attach(pid::setD);

    steer.burnFlash();
    Logger.recordOutput("Firmware/" + name + "_Steer", steer.getFirmwareString());
    Logger.recordOutput("Firmware/" + name + "_Thrust", thrust.getVersion().getValue());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(velSignal, posSignal, errSignal, tempSignal);
    inputs.pos = getModulePosition();
    inputs.vel =
        new SwerveModuleState(
            velSignal.getValue() * THRUST_DISTANCE_PER_TICK, getModulePosition().angle);
    inputs.steerTempC = steer.getMotorTemperature();
    inputs.thrustErr = errSignal.getValue();
    inputs.thrustTempC = tempSignal.getValue();
    inputs.offset = off;
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    double cmd_ang = state.angle.getRotations();
    thrust.setControl(new VelocityDutyCycle(state.speedMetersPerSecond / THRUST_DISTANCE_PER_TICK));

    pid.setReference(((cmd_ang % 1.0) + 1.0) % 1.0, CANSparkMax.ControlType.kSmartMotion);
  }

  private SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        posSignal.getValue() * THRUST_DISTANCE_PER_TICK,
        Rotation2d.fromRotations(encoder.getPosition()));
  }
}
