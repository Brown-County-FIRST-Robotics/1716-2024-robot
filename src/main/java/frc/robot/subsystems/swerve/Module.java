package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber minNoMotionTime =
      new LoggedTunableNumber("Min no motion time", 0.5);
  private static final LoggedTunableNumber maxMotionAllowed =
      new LoggedTunableNumber("Max motion", 0.05);
  ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  ModuleIO io;
  int ind;
  String name;
  Rotation2d chassisOffset;
  Rotation2d relativeSensorZeroPosition = new Rotation2d();
  Timer noMotionTimer = new Timer();

  public Module(ModuleIO io, int ind) {
    this.io = io;
    this.ind = ind;
    switch (ind) {
      case 0:
        chassisOffset = Rotation2d.fromDegrees(-90);
        name = "FL";
        break;
      case 1:
        chassisOffset = Rotation2d.fromDegrees(0);
        name = "FR";
        break;
      case 2:
        chassisOffset = Rotation2d.fromDegrees(180);
        name = "BL";
        break;
      case 3:
        chassisOffset = Rotation2d.fromDegrees(90);
        name = "BR";
        break;
    }
    periodic();
    reZero();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name + "_Inputs", inputs);
    if (Math.abs(inputs.absSensorOmega) > maxMotionAllowed.get()
        || Math.abs(inputs.relativeSensorOmega) > maxMotionAllowed.get()) {
      noMotionTimer.restart();
    }
    if (noMotionTimer.hasElapsed(minNoMotionTime.get())&&inputs.absSensorAngle<0.5) {
      reZero();
    }
  }

  private void reZero() {
    Logger.recordOutput(
        "Drive/" + name + "/relative_encoder_drift",
        getAbsEncoderPos().minus(getProcessedRelativeEncoderPos()).getDegrees());
    relativeSensorZeroPosition = getAbsEncoderPos().minus(getUnprocessedRelativeEncoderPos());
    noMotionTimer.restart();
  }

  private Rotation2d getProcessedRelativeEncoderPos() {
    return relativeSensorZeroPosition.plus(getUnprocessedRelativeEncoderPos());
  }

  private Rotation2d getUnprocessedRelativeEncoderPos() {
    return Rotation2d.fromRotations(inputs.relativeSensorAngle);
  }

  private Rotation2d getAbsEncoderPos() {
    return Rotation2d.fromRotations(inputs.absSensorAngle)
        .minus(Rotation2d.fromRotations(inputs.offset))
        .unaryMinus();
  }

  private Rotation2d getChassisRelativeRotation() {
    return getProcessedRelativeEncoderPos().minus(chassisOffset);
  }

  public SwerveModuleState getChassisRelativeState() {
    return new SwerveModuleState(inputs.thrustVel, getChassisRelativeRotation());
  }

  public SwerveModulePosition getChassisRelativePosition() {
    return new SwerveModulePosition(inputs.thrustPos, getChassisRelativeRotation());
  }

  public void setState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getChassisRelativeRotation());
    state.speedMetersPerSecond *= getChassisRelativeRotation().minus(state.angle).getCos();
    Rotation2d cmdPosForRelativeEncoder =
        state.angle.plus(chassisOffset).minus(relativeSensorZeroPosition);
    double adjustedRelCmd =
        inputs.relativeSensorAngle
            - (inputs.relativeSensorAngle % 1.0)
            + cmdPosForRelativeEncoder.getRotations();
    if (Math.abs(adjustedRelCmd - inputs.relativeSensorAngle)
        > Math.abs(1.0 + adjustedRelCmd - inputs.relativeSensorAngle)) {
      adjustedRelCmd += 1.0;
    }
    if (Math.abs(adjustedRelCmd - inputs.relativeSensorAngle)
        > Math.abs(-1.0 + adjustedRelCmd - inputs.relativeSensorAngle)) {
      adjustedRelCmd -= 1.0;
    }
    io.setCmdState(adjustedRelCmd, state.speedMetersPerSecond);
  }
}
