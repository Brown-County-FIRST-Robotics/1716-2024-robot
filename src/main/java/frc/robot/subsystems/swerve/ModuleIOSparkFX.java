package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ModuleIOSparkFX implements ModuleIO {
  private final double THRUST_DISTANCE_PER_TICK = .0254 * 4.0 * Math.PI / (2048.0 * 6.75);
  private final CANSparkMax steer;
  private final SparkMaxAnalogSensor encoder;
  private final SparkMaxPIDController pid;
  private final WPI_TalonFX thrust;
  String name;
  LoggedTunableNumber thrustP = new LoggedTunableNumber("Thrust P", 0);
  LoggedTunableNumber thrustI = new LoggedTunableNumber("Thrust I", 0);
  LoggedTunableNumber thrustD = new LoggedTunableNumber("Thrust D", 0);
  LoggedTunableNumber thrustKV =
      new LoggedTunableNumber("Thrust KV", 1023.0 * 600.0 / (6380.0 * 2048.0));
  LoggedTunableNumber steerP = new LoggedTunableNumber("Steer P", 0);
  LoggedTunableNumber steerI = new LoggedTunableNumber("Steer I", 0);
  LoggedTunableNumber steerD = new LoggedTunableNumber("Steer D", 0);
  LoggedTunableNumber steerKV = new LoggedTunableNumber("Steer KV", 1.0 / 300.0);
  Rotation2d steerOffset;
  Rotation2d chasisOffset;

  public ModuleIOSparkFX(int thrustID, int steerID, String name, Rotation2d chasisOffset) {
    this.name = name;
    this.chasisOffset = chasisOffset;
    thrust = new WPI_TalonFX(thrustID);
    thrust.setNeutralMode(NeutralMode.Coast);
    thrust.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    thrust.configNominalOutputForward(0, 20);
    thrust.configNominalOutputReverse(0, 20);
    thrust.configPeakOutputForward(1, 20);
    thrust.configPeakOutputReverse(-1, 20);

    steer = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);
    steer.restoreFactoryDefaults();
    pid = steer.getPIDController();
    encoder = steer.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

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

    reconfigure();

//    if (steerKV.hasChanged()) {
      pid.setP(steerP.get());
//    }
//    if (steerKV.hasChanged()) {
      pid.setI(steerI.get());
//    }
//    if (steerKV.hasChanged()) {
      pid.setD(steerD.get());
//    }
//    if (steerKV.hasChanged()) {
      pid.setFF(steerKV.get());
//    }
    
    steer.burnFlash();
    Logger.getInstance().recordMetadata(name + "_Steer_FW", steer.getFirmwareString());
    Logger.getInstance()
        .recordMetadata(name + "_Thrust_FW", String.valueOf(thrust.getFirmwareVersion()));
    Logger.getInstance().recordMetadata(name + "_Thrust_Name", thrust.getDescription());
    steerOffset = Rotation2d.fromRotations(thrust.configGetCustomParam(0) / 1000.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.steerPos = getModulePosition().angle.getRotations();
    inputs.thrustPos = getModulePosition().distanceMeters;
    inputs.steerTempC = steer.getMotorTemperature();
    inputs.thrustErr = thrust.getClosedLoopError();
    inputs.thrustTempC = thrust.getTemperature();
  }

  @Override
  public void setCmdState(SwerveModuleState state) {
    state.speedMetersPerSecond *= getModulePosition().angle.minus(state.angle).getCos();
    double cmd_ang = state.angle.plus(chasisOffset).unaryMinus().plus(steerOffset).getRotations();
    Logger.getInstance().recordOutput(name+"_ang",cmd_ang);

    thrust.set(
        TalonFXControlMode.Velocity, 0.1 * state.speedMetersPerSecond / THRUST_DISTANCE_PER_TICK);

    pid.setReference(((cmd_ang % 1.0) + 1.0) % 1.0, CANSparkMax.ControlType.kSmartMotion);
  }

  private SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        thrust.getSelectedSensorPosition() * THRUST_DISTANCE_PER_TICK,
        Rotation2d.fromRotations(encoder.getPosition()).minus(steerOffset).unaryMinus().minus(chasisOffset));
  }

  @Override
  public void reconfigure() {
    if (steerKV.hasChanged()) {
      pid.setP(steerP.get());
    }
    if (steerKV.hasChanged()) {
      pid.setI(steerI.get());
    }
    if (steerKV.hasChanged()) {
      pid.setD(steerD.get());
    }
    if (steerKV.hasChanged()) {
      pid.setFF(steerKV.get());
    }
    if (thrustP.hasChanged()) {
      thrust.config_kP(0, thrustP.get(), 20);
    }
    if (thrustI.hasChanged()) {
      thrust.config_kI(0, thrustI.get(), 20);
    }
    if (thrustD.hasChanged()) {
      thrust.config_kD(0, thrustD.get(), 20);
    }
    if (thrustKV.hasChanged()) {
      thrust.config_kF(0, thrustKV.get(), 20);
    }
  }
}
