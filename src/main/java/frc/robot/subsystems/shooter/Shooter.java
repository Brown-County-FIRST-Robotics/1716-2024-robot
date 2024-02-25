package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  ShooterIO shooterIO;
  FeederIO feederIO;
  FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  double cmdTopSpeed;
  double cmdBottomSpeed;
  LoggedTunableNumber speedThreshold = new LoggedTunableNumber("Shooting speed threshold", 0.05);
  LoggedTunableNumber firingTime = new LoggedTunableNumber("Firing Time", 0.5);

  boolean isShooting = false;
  public boolean isFiring = false;
  public boolean intaking = false;
  double feedCmd = 0.0;

  boolean holding = true;
  boolean firingBlocked = false;

  double firingStartTime;

  public void setFiringBlocked(boolean firingBlocked) {
    this.firingBlocked = firingBlocked;
  }

  public boolean isHolding() {
    return holding;
  }

  public void setHolding(boolean holding) {
    this.holding = holding;
  }

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
        && Math.abs((shooterInputs.velocity[0] - cmdTopSpeed) / cmdTopSpeed) < speedThreshold.get()
        && Math.abs((shooterInputs.velocity[1] - cmdBottomSpeed) / cmdBottomSpeed)
            < speedThreshold.get()
        && !firingBlocked) {
      isFiring = true;
      firingStartTime = Timer.getFPGATimestamp();
    }
    // The open and closed should always be opposite, and anything else would be an electrical fault
    if (intaking && (feederInputs.closedContact == feederInputs.openContact)) {
      intaking = false;
      // Shut down to prevent damage to ring
      setFeeder(0);
      cmdVel(0, 0);
      holding = true;
      System.out.println("Feeder limit switch disconnected!!");
    }
    if (intaking && feederInputs.closedContact) {
      setFeeder(0);
      cmdVel(0, 0);
      holding = true;
      intaking = false;
    }
    if (isFiring) {
      setFeeder(-8000);
      if (firingStartTime + firingTime.get() < Timer.getFPGATimestamp()) {
        isFiring = false;
        isShooting = false;
        holding = false;
        setFeeder(0);
      }
    }
  }

  public void cmdVel(double v1, double v2) {
    shooterIO.setVelocity(v1, v2);
  }

  public void setSpeed(double exitVel) {
    double factor = 3500 / 9.88;
    cmdTopSpeed = -factor * exitVel;
    cmdBottomSpeed = factor * exitVel;
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

  public void setFeeder(double vel) {
    feedCmd = vel;
    feederIO.setVel(feedCmd);
  }
}
