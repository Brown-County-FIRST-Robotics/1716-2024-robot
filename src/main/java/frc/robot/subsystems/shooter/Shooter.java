package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  ShooterIO shooterIO;
  FeederIO feederIO;
  double lastFeederCMD = 0.0;
  FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  LoggedTunableNumber preset_HOLD = new LoggedTunableNumber("Shooter/HOLDING_preset", 0.0);
  LoggedTunableNumber preset_INTAKE_OR_SHOOT =
      new LoggedTunableNumber("Shooter/FEEDING_TO_SHOOTER_preset", 4.0);
  // TEMP CODE
  LoggedDashboardNumber topShootingSpeed = new LoggedDashboardNumber("Top Shooting RPM", 6500);
  LoggedDashboardNumber bottomShootingSpeed =
      new LoggedDashboardNumber("Bottom Shooting RPM", -6500);
  // END TEMP
  LoggedTunableNumber speedThreshold = new LoggedTunableNumber("Shooting speed threshold", 0.05);
  LoggedTunableNumber firingTime = new LoggedTunableNumber("Firing Time", 0.5);

  boolean isShooting = false;
  boolean isFiring = false;
  double firingStartTime;
  FeederPreset currentPreset = FeederPreset.INTAKE_OR_SHOOT;

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
    if (isShooting) {
      shooterIO.setVelocity(topShootingSpeed.get(), bottomShootingSpeed.get());
    } else {
      shooterIO.setVelocity(0, 0);
    }
    if (!isFiring
        && isShooting
        && Math.abs((shooterInputs.velocity[0] + topShootingSpeed.get()) / topShootingSpeed.get())
            < speedThreshold.get()
        && Math.abs(
                (shooterInputs.velocity[1] - bottomShootingSpeed.get()) / bottomShootingSpeed.get())
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

    checkFeeder();
  }

  // Checks the feeder position to see if a note has been inserted and bumped the feeder
  public void checkFeeder() {
    if (currentPreset == FeederPreset.INTAKE_OR_SHOOT && feederInputs.beamBroken) { //TODO: DOES THIS NEED A DELAY?
      cmdFeeder(FeederPreset.HOLD);
      currentPreset = FeederPreset.HOLD;
    }
  }

  public void cmdVel(double v1,double v2) {
    shooterIO.setVelocity(v1,v2);
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
    feederIO.setPosition(cmd);
  }

  public static enum FeederPreset {
    INTAKE_OR_SHOOT,
    HOLD
  }
}
