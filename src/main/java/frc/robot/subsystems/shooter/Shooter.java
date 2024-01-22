package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  ShooterIO shooterIO;
  FeederIO feederIO;
  FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  LoggedDashboardBoolean shouldReset = new LoggedDashboardBoolean("Reset arm", false);
  LoggedDashboardNumber feederPos = new LoggedDashboardNumber("Arm Pos", 0);

  public Shooter(ShooterIO io, FeederIO feederIO) {
    this.shooterIO = io;
    this.feederIO = feederIO;
    io.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/ShooterInputs", shooterInputs);
    feederIO.updateInputs(feederInputs);
    Logger.processInputs("Shooter/FeederInputs", feederInputs);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/ShooterInputs", shooterInputs);
    feederIO.updateInputs(feederInputs);
    Logger.processInputs("Shooter/FeederInputs", feederInputs);
    if (shouldReset.get()) {
      feederIO.resetPos();
      shouldReset.set(false);
    }
    feederIO.cmdPos(feederPos.get());
  }

  public void cmdvel(double voltage) {
    shooterIO.setVelocity(voltage);
  }

  public void cmdVoltage(double voltage) {
    shooterIO.setVoltage(voltage);
  }
}
