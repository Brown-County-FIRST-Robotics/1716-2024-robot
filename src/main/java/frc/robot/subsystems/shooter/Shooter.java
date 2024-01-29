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
  LoggedTunableNumber preset_HOLD = new LoggedTunableNumber("Shooter/HOLDING_preset", 0.0);
  LoggedTunableNumber preset_INTAKE_OR_SHOOT =
      new LoggedTunableNumber("Shooter/FEEDING_TO_SHOOTER_preset", 4.0);
  TrapezoidProfile.Constraints feederConstraints =
      new TrapezoidProfile.Constraints(7.0 * 5700 / 60, 3 * 7.0 * 5700 / 60);
  SimpleMotorFeedforward feederFF =
      new SimpleMotorFeedforward(-0.3, 1.1 * 12.0 * 60 / (7.0 * 5700));
  LoggedDashboardNumber shootingSpeed = new LoggedDashboardNumber("Shooting RPM", 6500);
  LoggedTunableNumber speedThreshold = new LoggedTunableNumber("Shooting speed threshold", 0.05);
  LoggedTunableNumber firingTime = new LoggedTunableNumber("Firing Time", 0.5);

  boolean isShooting = false;
  boolean isFiring = false;
  double firingStartTime;
  PIDController pidController = new PIDController(1, 0.1, 0);
  FeederPreset currentPreset = FeederPreset.INTAKE_OR_SHOOT;

  public Shooter(ShooterIO io, FeederIO feederIO) {
    this.shooterIO = io;
    this.feederIO = feederIO;
    io.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/ShooterInputs", shooterInputs);
    feederIO.updateInputs(feederInputs);
    Logger.processInputs("Shooter/FeederInputs", feederInputs);
    new LoggedTunableNumber("P", 0.8).attach(pidController::setP);
    new LoggedTunableNumber("I", 0).attach(pidController::setI);
    new LoggedTunableNumber("D", 0).attach(pidController::setD);
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
      cmdFeeder(FeederPreset.INTAKE_OR_SHOOT);
      currentPreset = FeederPreset.INTAKE_OR_SHOOT;
      if (firingStartTime + firingTime.get() < Timer.getFPGATimestamp()) {
        isFiring = false;
        isShooting = false;
      }
    }
    feederIO.setVoltage(pidController.calculate(feederInputs.position, lastFeederCMD));

    checkFeeder();
  }

  //Checks the feeder position to see if a note has been inserted and bumped the feeder
  public void checkFeeder() {
    if (currentPreset == FeederPreset.INTAKE_OR_SHOOT && Math.abs(feederInputs.position - preset_INTAKE_OR_SHOOT.get()) > 0.02) { //Margin for error here
      cmdFeeder(FeederPreset.HOLD);
      currentPreset = FeederPreset.HOLD;
    }
  }

  public void cmdVel(double voltage) {
    shooterIO.setVelocity(voltage);
  }

  public void cmdVoltage(double voltage) {
    shooterIO.setVoltage(voltage);
  }

  public void shoot() {
    isShooting = true;
  }

  public void cmdFeeder(FeederPreset preset) {
    switch (preset) {
      case INTAKE_OR_SHOOT:
        cmdFeeder(preset_INTAKE_OR_SHOOT.get());
        currentPreset = FeederPreset.INTAKE_OR_SHOOT;
        break;
      case HOLD:
        cmdFeeder(preset_HOLD.get());
        currentPreset = FeederPreset.HOLD;
        break;
    }
  }

  private void cmdFeeder(double cmd) {
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

  public static enum FeederPreset {
    INTAKE_OR_SHOOT,
    HOLD
  }
}
