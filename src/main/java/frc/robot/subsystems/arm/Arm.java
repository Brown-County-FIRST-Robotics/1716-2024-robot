package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  Mechanism2d realStates = new Mechanism2d(100, 100);
  MechanismLigament2d realArmStates = new MechanismLigament2d("Arm", 40, 0);
  Mechanism2d cmdStates = new Mechanism2d(100, 100);
  MechanismLigament2d cmdArmStates = new MechanismLigament2d("Arm", 40, 0);
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    realStates.getRoot("Root", 50, 50).append(realArmStates);
    cmdStates.getRoot("Root", 50, 50).append(cmdArmStates);
    Logger.recordOutput("Arm/realState", realStates);
    Logger.recordOutput("Arm/cmdState", cmdStates);
    this.io = io;
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Inputs", inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm/Inputs", inputs);
  }
}
