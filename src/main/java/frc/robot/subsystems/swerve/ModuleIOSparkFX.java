package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ModuleIOSparkFX implements ModuleIO {
  private final double THRUST_DISTANCE_PER_TICK = .0254 * 4.0 * Math.PI / 6.75;
  private final CANSparkMax steer;
  private final SparkAnalogSensor encoder;
  private final SparkPIDController pid;
  private final TalonFX thrust;
  String name;
  LoggedTunableNumber thrustP = new LoggedTunableNumber("Thrust P", 0);
  LoggedTunableNumber thrustI = new LoggedTunableNumber("Thrust I", 0);
  LoggedTunableNumber thrustD = new LoggedTunableNumber("Thrust D", 0);
  LoggedTunableNumber thrustKV = new LoggedTunableNumber("Thrust KV", 60.0 / 6380.0);
  LoggedTunableNumber steerP = new LoggedTunableNumber("Steer P", 0);
  LoggedTunableNumber steerI = new LoggedTunableNumber("Steer I", 0);
  LoggedTunableNumber steerD = new LoggedTunableNumber("Steer D", 0);
  LoggedTunableNumber steerKV = new LoggedTunableNumber("Steer KV", 1.0 / 300.0);

  public ModuleIOSparkFX(int thrustID, int steerID, String name) {
    this.name = name;
    thrust = new TalonFX(thrustID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kV = thrustKV.get();
    if (name == "FL") {
      config.CustomParams.CustomParam0 = 825;
    } else if (name == "FR") {
      config.CustomParams.CustomParam0 = 5;
    } else if (name == "BL") {
      config.CustomParams.CustomParam0 = 982;
    } else if (name == "BR") {
      config.CustomParams.CustomParam0 = 456;
    }
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    thrust.getConfigurator().apply(config);
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
    steer.setSmartCurrentLimit(30);

    steerKV.attach(pid::setFF);
    steerP.attach(pid::setP);
    steerI.attach(pid::setI);
    steerD.attach(pid::setD);

    steer.burnFlash();
    Logger.recordOutput(name + "_Steer_FW", steer.getFirmwareString());
    Logger.recordOutput(name + "_Thrust_Name", thrust.getDescription());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.pos = getModulePosition();
    inputs.vel =
        new SwerveModuleState(
            thrust.getRotorVelocity().getValue() * THRUST_DISTANCE_PER_TICK,
            getModulePosition().angle);
    inputs.steerTempC = steer.getMotorTemperature();
    inputs.thrustErr = thrust.getClosedLoopError().getValue();
    inputs.thrustTempC = thrust.getDeviceTemp().getValue();
    //    inputs.offset = thrust.configGetCustomParam(0) / 1000.0;
    if (name == "FL") {
      inputs.offset = 0.825;
    } else if (name == "FR") {
      inputs.offset = 0.005;
    } else if (name == "BL") {
      inputs.offset = 0.982;
    } else if (name == "BR") {
      inputs.offset = 0.456;
    }
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    double cmd_ang = state.angle.getRotations();
    thrust.setControl(new VelocityVoltage(state.speedMetersPerSecond / THRUST_DISTANCE_PER_TICK));

    pid.setReference(((cmd_ang % 1.0) + 1.0) % 1.0, CANSparkMax.ControlType.kSmartMotion);
  }

  private SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        thrust.getRotorPosition().getValue() * THRUST_DISTANCE_PER_TICK,
        Rotation2d.fromRotations(encoder.getPosition()));
  }
}
