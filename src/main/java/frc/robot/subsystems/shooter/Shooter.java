package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  ShooterIO shooterIO;
  FeederIO feederIO;
  double lastFeederCMD = 0.0;
  FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  LoggedDashboardBoolean shouldReset = new LoggedDashboardBoolean("Reset arm", false);
  LoggedTunableNumber preset_RECEIVING_FROM_INTAKE =
      new LoggedTunableNumber("Shooter/RECEIVING_FROM_INTAKE_preset", 0.0);
  LoggedTunableNumber preset_HOLDING = new LoggedTunableNumber("Shooter/HOLDING_preset", 0.0);
  LoggedTunableNumber preset_FEEDING_TO_SHOOTER =
      new LoggedTunableNumber("Shooter/FEEDING_TO_SHOOTER_preset", 0.0);
  TrapezoidProfile.Constraints feederConstraints =
      new TrapezoidProfile.Constraints(7.0 * 5700 / 60, 3 * 7.0 * 5700 / 60);
  SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(-0.3, 1.1*12.0 * 60 / (7.0 * 5700));
  LoggedDashboardNumber shootingSpeed = new LoggedDashboardNumber("Shooting RPM", 6500);
  LoggedTunableNumber speedThreshold = new LoggedTunableNumber("Shooting speed threshold", 0.05);
  LoggedTunableNumber firingTime = new LoggedTunableNumber("Firing Time", 0.5);

  boolean isShooting = false;
  boolean isFiring = false;
  double firingStartTime;
  PIDController pc=new PIDController(1,0.1,0);

  public Shooter(ShooterIO io, FeederIO feederIO) {
    this.shooterIO = io;
    this.feederIO = feederIO;
    io.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/ShooterInputs", shooterInputs);
    feederIO.updateInputs(feederInputs);
    Logger.processInputs("Shooter/FeederInputs", feederInputs);
    new LoggedTunableNumber("P",1).attach(pc::setP);
    new LoggedTunableNumber("I",0.1).attach(pc::setI);
    new LoggedTunableNumber("D",0).attach(pc::setD);
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
    if (isShooting) {
      shooterIO.setVelocity(shootingSpeed.get());
    } else {
      shooterIO.setVelocity(0);
    }
    if (!isFiring
        && isShooting
        && Math.abs((shooterInputs.velocity[0] + shootingSpeed.get()) / shootingSpeed.get())
            < speedThreshold.get()
        && Math.abs((shooterInputs.velocity[1] - shootingSpeed.get()) / shootingSpeed.get())
            < speedThreshold.get()) {
      isFiring = true;
      firingStartTime = Timer.getFPGATimestamp();
    }
    if (isFiring) {
      cmdFeeder(FeederPreset.FEEDING_TO_SHOOTER);
      if (firingStartTime + firingTime.get() < Timer.getFPGATimestamp()) {
        isFiring = false;
        isShooting = false;
      }
    }
    feederIO.setVoltage(pc.calculate(feederInputs.position,lastFeederCMD));

  }

  public void cmdvel(double voltage) {
    shooterIO.setVelocity(voltage);
  }

  public void cmdVoltage(double voltage) {
    shooterIO.setVoltage(voltage);
  }

  public void cmdFeeder(double cmd) {
    lastFeederCMD = cmd;
    feederIO.setVoltage(
        feederFF.calculate(
            new TrapezoidProfile(feederConstraints)
                .calculate(
                    0.02,
                    new TrapezoidProfile.State(feederInputs.position, feederInputs.velocity),
                    new TrapezoidProfile.State(lastFeederCMD, 0))
                .velocity,
            0));
  }

  public void shoot() {
    isShooting = true;
  }

  public void cmdFeeder(FeederPreset preset) {
    switch (preset) {
      case HOLDING:
        cmdFeeder(preset_HOLDING.get());
        break;
      case RECEIVING_FROM_INTAKE:
        cmdFeeder(preset_RECEIVING_FROM_INTAKE.get());
        break;
      case FEEDING_TO_SHOOTER:
        cmdFeeder(preset_FEEDING_TO_SHOOTER.get());
        break;
    }
  }

  public static enum FeederPreset {
    RECEIVING_FROM_INTAKE,
    HOLDING,
    FEEDING_TO_SHOOTER
  }
}
