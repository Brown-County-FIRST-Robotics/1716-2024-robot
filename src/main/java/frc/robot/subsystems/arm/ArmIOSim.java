package frc.robot.subsystems.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

/** A simulated arm */
public class ArmIOSim implements ArmIO {
  SlewRateLimiter armSRL = new SlewRateLimiter(3, -5, 0);
  Rotation2d cmdAngle = new Rotation2d();

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angle = Rotation2d.fromRotations(armSRL.calculate(cmdAngle.getRotations()));
  }

  @Override
  public void setAngle(Rotation2d cmdAng, double arbFF) {
    cmdAngle = cmdAng;
  }
}
