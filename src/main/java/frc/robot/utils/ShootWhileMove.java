package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShootWhileMove {
  public interface ShooterKinematics {
    Translation3d getPose(ShootingCommand cmd, Translation2d botPose);
  }

  public static class ShootingCommand {
    public Rotation2d botAngle;
    public Rotation2d shooterAngle;
    public double shooterSpeedMPS;

    public ShootingCommand(Rotation2d botAngle, Rotation2d shooterAngle, double shooterSpeedMPS) {
      this.botAngle = botAngle;
      this.shooterAngle = shooterAngle;
      this.shooterSpeedMPS = shooterSpeedMPS;
    }
  }

  private static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  public static ShootingCommand calcSimpleCommand(
      Translation3d botPose, Translation2d botVelocity) {
    double g = 9.8065;
    double tmpA = Math.sqrt(-g * botPose.getZ());
    Translation2d tmpB = botPose.toTranslation2d().times(-g / tmpA).minus(botVelocity);
    double shooterVel = Math.sqrt(dot(tmpB, tmpB) - g * botPose.getZ());
    return new ShootingCommand(
        tmpB.getAngle(), Rotation2d.fromRadians(Math.asin(tmpA / shooterVel)), shooterVel);
  }

  public static ShootingCommand calcCommandWithKinematics(
      Translation2d botPose,
      Translation3d target,
      Translation2d botVelocity,
      ShooterKinematics kinematics) {
    ShootingCommand lastCommand = new ShootingCommand(new Rotation2d(), new Rotation2d(), 0);
    for (int i = 0; i < 100; i++) {
      Translation3d poseOfBot = kinematics.getPose(lastCommand, botPose);
      lastCommand = calcSimpleCommand(poseOfBot.minus(target), botVelocity);
    }
    return lastCommand;
  }
}
