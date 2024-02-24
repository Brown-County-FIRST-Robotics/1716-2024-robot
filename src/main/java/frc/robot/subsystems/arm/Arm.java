package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** The Arm subsystem */
public class Arm extends SubsystemBase {
  Mechanism2d realStates = new Mechanism2d(100, 100);
  MechanismLigament2d realArmStates = new MechanismLigament2d("Arm", 40, 0);
  Mechanism2d cmdStates = new Mechanism2d(100, 100);
  MechanismLigament2d cmdArmStates = new MechanismLigament2d("Arm", 40, 0);
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  Rotation2d cmdAng = new Rotation2d();
  LoggedTunableNumber gravFF = new LoggedTunableNumber("Arm Gravity FF", 0.0);
  LoggedTunableNumber neutralPosition = new LoggedTunableNumber("Arm neutral position", 0.0);

  /**
   * Constructs the subsystem from an IO object
   *
   * @param io The IO interface to use
   */
  public Arm(ArmIO io) {
    realStates.getRoot("Root", 50, 50).append(realArmStates);
    cmdStates.getRoot("Root", 50, 50).append(cmdArmStates);
    Logger.recordOutput("Arm/realState", realStates);
    Logger.recordOutput("Arm/cmdState", cmdStates);
    this.io = io;
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Inputs", inputs);
  }

  /**
   * Commands an angle
   *
   * @param rot The angle set point
   */
  public void setAngle(Rotation2d rot) {
    cmdArmStates.setAngle(rot);
    Logger.recordOutput("Arm/cmdState", cmdStates);
    cmdAng = rot;
    commandAngle();
  }

  private void commandAngle() {
    io.setAngle(cmdAng, cmdAng.getCos() * gravFF.get());
  }

  public void commandNeutral() {
    setAngle(Rotation2d.fromRotations(neutralPosition.get()));
  }
  /**
   * Gets the angle of the arm
   *
   * @return The angle of the arm
   */
  public Rotation2d getAngle() {
    return inputs.angle;
  }

  public double getOmega() {
    return inputs.omega;
  }

  /**
   * Increments the angle of the arm. This commands an angle based on the current angle of the arm,
   * to prevent windup.
   *
   * @param increment The amount to increment
   */
  public void commandIncrement(Rotation2d increment) {
    setAngle(getAngle().plus(increment));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Inputs", inputs);
    commandAngle();
    cmdArmStates.setAngle(cmdAng);
    realArmStates.setAngle(getAngle());
    Logger.recordOutput("Arm/realState", realStates);
    Logger.recordOutput("Arm/cmdState", cmdStates);
    var bp = new Pose3d(-0.3, 0.33, 0.65, new Rotation3d());
    Logger.recordOutput(
        "Arm/CmdMech3d",
        bp.plus(
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, -Math.PI / 2, Math.PI / 2)
                    .rotateBy(new Rotation3d(0, cmdAng.getRadians(), 0)))));
    Logger.recordOutput(
        "Arm/RealMech3d",
        bp.plus(
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, -Math.PI / 2, Math.PI / 2)
                    .rotateBy(new Rotation3d(0, getAngle().getRadians(), 0)))));
  }
}
