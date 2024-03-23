package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Utility functions for calculating shooter angles */
public class ShootWhileMove {
  static CustomAlerts.TimeLatchAlert nanLatch =
      new CustomAlerts.TimeLatchAlert(
          Alert.AlertType.ERROR, 2.0, "You are too far away (NaN arm goal)");
  /** The kinematics of the shooter */
  public interface ShooterKinematics {
    /**
     * Gets the pose of the tip of the shooter given a command
     *
     * @param cmd The shooting command
     * @param botPose The 2d robot pose
     * @return The position of the tip of the shooter
     */
    Translation3d getPose(ShootingCommand cmd, Translation2d botPose);
  }

  /** Predictor for robot motion and shooter delays */
  public interface SystemPredictor {
    /**
     * Calculates the time to fire given a state
     *
     * @param cmd The state
     * @return The time to fire at that state
     */
    double getTimeToState(ShootingCommand cmd);

    /**
     * Gets the robot pose at a future time
     *
     * @param t The time at which to get the pose
     * @return The pose at that time
     */
    Translation2d getPoseAfterTime(double t);

    /**
     * Gets the robot velocity at a future time
     *
     * @param t The time at which to get the velocity
     * @return The velocity at that time
     */
    Translation2d getBotVelAfterTime(double t);
  }

  /**
   * Converts robot-relative speeds to field-relative
   *
   * @param speeds The robot-relative speeds
   * @param angle The angle of the robot
   * @return The field-relative speeds
   */
  public static Translation2d getFieldRelativeSpeeds(ChassisSpeeds speeds, Rotation2d angle) {
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(angle.unaryMinus());
  }

  /**
   * Converts a velocity over dt seconds to a twist
   *
   * @param speeds The robot speeds
   * @param dt The amount of time elapsed
   * @return The displacement during that interval
   */
  public static Twist2d discreteSpeeds(ChassisSpeeds speeds, double dt) {
    return new Twist2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        speeds.omegaRadiansPerSecond * dt);
  }

  /** A command to give to the shooter */
  public static class ShootingCommand {
    /** The angle of the robot */
    public Rotation2d botAngle;
    /** The angle of the shooter */
    public Rotation2d shooterAngle;

    public ShootingCommand(Rotation2d botAngle, Rotation2d shooterAngle) {
      this.botAngle = botAngle;
      this.shooterAngle = shooterAngle;
    }
  }

  public static ShootingCommand calcSimpleCommand(
      Translation3d target, Translation3d botPose, Translation2d botVel) {
    double g = 9.8065;
    double v = 10;
    var pbmt = target.minus(botPose);
    double px = pbmt.getX();
    double py = pbmt.getY();
    double pz = pbmt.getZ();
    double bx = botVel.getX();
    double by = botVel.getY();

    double theta_s = Math.asin(Math.sqrt(2 * g * pz) / v) * 1.01;
    double theta_b = Math.atan2(py, px);
    for (int i = 0; i < 20; i++) {
      double vz = v * Math.sin(theta_s);
      double vground = v * Math.cos(theta_s);
      double vx = vground * Math.cos(theta_b) + bx;
      double vy = vground * Math.sin(theta_b) + by;
      double t = (vz - Math.sqrt(vz * vz - 2 * g * pz)) / g;

      double ex = vx * t - px;
      double ey = vy * t - py;
      double dtdts =
          (vground - (v * vground * Math.sin(theta_s) / Math.sqrt(vz * vz - 2 * g * pz))) / g;

      double dexdtb = -t * vground * Math.sin(theta_b);
      double deydtb = t * vground * Math.cos(theta_b);

      double dexdts = dtdts * vx - t * vz * Math.cos(theta_b);
      double deydts = dtdts * vy - t * vz * Math.sin(theta_b);

      double det = dexdtb * deydts - dexdts * deydtb;
      double deltb = (deydts * ex - dexdts * ey) / det;
      double delts = (-deydtb * ex + dexdtb * ey) / det;
      if (Math.abs(delts) + Math.abs(deltb) < 0.001) {
        break;
      }
      theta_s = theta_s - delts;
      theta_b = theta_b - deltb;
    }
    if (Double.isNaN(theta_s)) {
      nanLatch.latch();
      theta_s=0;
    }
    return new ShootingCommand(Rotation2d.fromRadians(theta_b), Rotation2d.fromRadians(theta_s));
  }

  /**
   * Checks if the difference between the two commands if enough to stop iteration
   *
   * @param a The last command
   * @param b The current command
   * @return If iteration can be stopped
   */
  private static boolean converged(ShootingCommand a, ShootingCommand b) {
    return Math.abs(a.botAngle.minus(b.botAngle).getDegrees()) < 0.1
        && Math.abs(a.shooterAngle.minus(b.shooterAngle).getDegrees()) < 0.1;
  }

  /**
   * Calculates a command that includes shooter kinematics
   *
   * @param botPose The pose of the robot on the field
   * @param target The pose of the target on the field
   * @param botVelocity The velocity of the robot
   * @param kinematics The shooter kinematics
   * @return The ideal angles and velocities
   */
  public static ShootingCommand calcCommandWithKinematics(
      Translation2d botPose,
      Translation3d target,
      Translation2d botVelocity,
      ShooterKinematics kinematics) {
    ShootingCommand lastCommand = new ShootingCommand(new Rotation2d(), new Rotation2d());
    for (int i = 0; i < 100; i++) {
      Translation3d poseOfBot = kinematics.getPose(lastCommand, botPose);
      var canidateCmd = calcSimpleCommand(target, poseOfBot, botVelocity);
      if (converged(canidateCmd, lastCommand)) {
        System.out.println("calcCommandWithKinematics converged in " + i + " iterations");
        return canidateCmd;
      }
      lastCommand = canidateCmd;
    }
    System.err.println("calcCommandWithKinematics did not converge");
    return lastCommand;
  }

  /**
   * Calculates a shooting command using kinematics and prediction
   *
   * @param predictor The predictor for the shooter
   * @param kinematics The shooter kinematics
   * @param target The pose of the target
   * @return The ideal shooting command
   */
  public static ShootingCommand calcCommandWithPrediction(
      SystemPredictor predictor, ShooterKinematics kinematics, Translation3d target) {
    double lastT = 0;
    ShootingCommand lastCommand = new ShootingCommand(new Rotation2d(), new Rotation2d());
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

  public static void main(String[] args) {}
}
