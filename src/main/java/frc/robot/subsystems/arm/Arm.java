package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  Mechanism2d realStates = new Mechanism2d(100, 100);
  MechanismLigament2d realArmStates = new MechanismLigament2d("Arm", 40, 0);
  Mechanism2d cmdStates = new Mechanism2d(100, 100);
  MechanismLigament2d cmdArmStates = new MechanismLigament2d("Arm", 40, 0);
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  Rotation2d cmdAng = new Rotation2d();
  LoggedTunableNumber gravFF = new LoggedTunableNumber("Arm Gravity FF", 0.0);

  public Arm(ArmIO io) {
    realStates.getRoot("Root", 50, 50).append(realArmStates);
    cmdStates.getRoot("Root", 50, 50).append(cmdArmStates);
    Logger.recordOutput("Arm/realState", realStates);
    Logger.recordOutput("Arm/cmdState", cmdStates);
    this.io = io;
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Inputs", inputs);
  }

  public void setAngle(Rotation2d rot) {
    cmdArmStates.setAngle(rot);
    Logger.recordOutput("Arm/cmdState", cmdStates);
    cmdAng = rot;
    commandAngle();
  }

  private void commandAngle() {
    io.setAngle(cmdAng, cmdAng.getCos() * gravFF.get());
  }

  public Rotation2d getAngle() {
    return inputs.angle;
  }

  public void commandIncrement(Rotation2d inc) {
    setAngle(getAngle().plus(inc));
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
  }
}
