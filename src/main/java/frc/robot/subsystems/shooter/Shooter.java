package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  ShooterIO shooterIO;
  FeederIO feederIO;
  FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  // TEMP CODE
  LoggedDashboardNumber topShootingSpeed = new LoggedDashboardNumber("Top Shooting RPM", 6500);
  LoggedDashboardNumber bottomShootingSpeed =
      new LoggedDashboardNumber("Bottom Shooting RPM", -6500);
  // END TEMP
  double cmdTopSpeed;
  double cmdBottomSpeed;
  LoggedTunableNumber speedThreshold = new LoggedTunableNumber("Shooting speed threshold", 0.05);
  LoggedTunableNumber firingTime = new LoggedTunableNumber("Firing Time", 0.5);

  boolean isShooting = false;
  boolean isFiring = false;
  double feedCmd=0.0;
  private LoggedTunableNumber ltn=new LoggedTunableNumber("shoor fac",4000);

  public boolean isHolding() {
    return holding;
  }

  boolean holding = false;

  public void setFiringBlocked(boolean firingBlocked) {
    this.firingBlocked = firingBlocked;
  }

  boolean firingBlocked = false;

  double firingStartTime;

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
      shooterIO.setVelocity(cmdTopSpeed, cmdBottomSpeed);
    } else {
      shooterIO.setVelocity(0, 0);
    }
    if (!isFiring
        && isShooting
        && Math.abs((shooterInputs.velocity[0] + cmdTopSpeed) / cmdTopSpeed) < speedThreshold.get()
        && Math.abs((shooterInputs.velocity[1] - cmdBottomSpeed) / cmdBottomSpeed)
            < speedThreshold.get()
        && !firingBlocked) {
      isFiring = true;
      firingStartTime = Timer.getFPGATimestamp();
    }
    if (isFiring) {
      cmdFeeder(8000);
      if (firingStartTime + firingTime.get() < Timer.getFPGATimestamp()) {
        isFiring = false;
        isShooting = false;
        holding = false;
        cmdFeeder(0);
      }
    }

  }



  public void cmdVel(double v1, double v2) {
    shooterIO.setVelocity(v1, v2);
  }

  public void cmdVoltage(double voltage) {
    shooterIO.setVoltage(voltage);
  }

  public void commandSpeed(double exitVel) {
    isShooting = true;
    holding = true;
    double factor = ltn.get() / 9.88;
    cmdTopSpeed = -factor * exitVel;
    cmdBottomSpeed = factor * exitVel;
  }

  public void shoot() {
    shoot(topShootingSpeed.get(), bottomShootingSpeed.get());
  }

  public void shoot(double tvel, double bvel) {
    holding = true;
    isShooting = true;
    cmdTopSpeed = tvel;
    cmdBottomSpeed = bvel;
  }


  public void stop() {
    setFiringBlocked(false);
    isShooting = false;
    isFiring = false;
    feederIO.setVel(0);
  }
  public void cmdFeeder(double vel){
    feedCmd=vel;
    feederIO.setVel(feedCmd);
  }



}
