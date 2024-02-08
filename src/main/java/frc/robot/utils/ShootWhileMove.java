package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootWhileMove {
  public interface ShooterKinematics {
    Translation3d getPose(ShootingCommand cmd, Translation2d botPose);
  }

  public interface SystemPredictor {
    double getTimeToState(ShootingCommand cmd);

    Translation2d getPoseAfterTime(double t);

    Translation2d getBotVelAfterTime(double t);
  }

  public static Translation2d getFieldRelativeSpeeds(ChassisSpeeds speeds, Rotation2d angle) {
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(angle.unaryMinus());
  }

  public static Twist2d discreteSpeeds(ChassisSpeeds speeds, double dt) {
    return new Twist2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        speeds.omegaRadiansPerSecond * dt);
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

  private static boolean converged(ShootingCommand a, ShootingCommand b) {
    return Math.abs(a.shooterSpeedMPS - b.shooterSpeedMPS) < 0.01
        && Math.abs(a.botAngle.minus(b.botAngle).getRotations()) < 0.01
        && Math.abs(a.shooterAngle.minus(b.shooterAngle).getRotations()) < 0.01;
  }

  public static ShootingCommand calcSimpleCommand(
      Translation3d botPose, Translation2d botVelocity) {
    double g = 9.8065;
    double tmpA = Math.sqrt(-g * botPose.getZ() * 2);
    Translation2d tmpB = botPose.toTranslation2d().times(-g / tmpA).minus(botVelocity);
    double shooterVel = Math.sqrt(dot(tmpB, tmpB) - g * botPose.getZ() * 2);
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
      var canidateCmd = calcSimpleCommand(poseOfBot.minus(target), botVelocity);
      if (converged(canidateCmd, lastCommand)) {
        System.out.println("calcCommandWithKinematics converged in " + i + " iterations");
        return canidateCmd;
      }
      lastCommand = canidateCmd;
    }
    System.out.println("calcCommandWithKinematics did not converge");
    return lastCommand;
  }

  public static ShootingCommand calcCommandWithPrediction(
      SystemPredictor predictor, ShooterKinematics kinematics, Translation3d target) {
    double lastT = 0;
    ShootingCommand lastCommand = new ShootingCommand(new Rotation2d(), new Rotation2d(), 0);
    for (int i = 0; i < 100; i++) {
      var canidateCmd =
          calcCommandWithKinematics(
              predictor.getPoseAfterTime(lastT),
              target,
              predictor.getBotVelAfterTime(lastT),
              kinematics);
      if (converged(canidateCmd, lastCommand)) {
        System.out.println("calcCommandWithPrediction converged in " + i + " iterations");
        return canidateCmd;
      }
      lastCommand = canidateCmd;
      lastT = predictor.getTimeToState(lastCommand);
    }
    System.out.println("calcCommandWithPrediction did not converge");
    return lastCommand;
  }

  public static void main(String[] args) { // Basic testing script
    Translation3d target = new Translation3d(0.458597, 5.544566, 2.1105114);
    Translation3d botPose = new Translation3d(3, 5, 0);
    Translation2d botvel = new Translation2d(0, 0);
    var cmd = calcSimpleCommand(botPose.minus(target), botvel);
    System.out.println(cmd.shooterSpeedMPS);
    Translation3d pose = botPose;
    Translation3d vel = new Translation3d(botvel.getX(), botvel.getY(), 0);
    double tVel = cmd.shooterSpeedMPS * cmd.shooterAngle.getCos();
    vel =
        vel.plus(
            new Translation3d(
                tVel * cmd.botAngle.getCos(),
                tVel * cmd.botAngle.getSin(),
                cmd.shooterSpeedMPS * cmd.shooterAngle.getSin()));
    double dt = 0.001;
    for (int i = 0; i < 10 / dt; i++) {
      pose = pose.plus(vel.times(dt));
      vel = vel.plus(new Translation3d(0, 0, -9.8065 * dt));
      if (pose.minus(target).getNorm() < 0.1) {
        System.out.println(
            "t:"
                + i * dt
                + "\tx:"
                + pose.getX()
                + "\ty:"
                + pose.getY()
                + "\tz:"
                + pose.getZ()
                + "\tvz:"
                + vel.getZ());
      }
    }
    System.out.println(9.88 > cmd.shooterSpeedMPS);
  }
}
