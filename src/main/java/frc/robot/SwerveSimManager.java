package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveSimManager {
  private static final double D = 21.125 * 0.0254; // TODO: Rename this
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(D / 2, D / 2),
          new Translation2d(D / 2, -D / 2),
          new Translation2d(-D / 2, D / 2),
          new Translation2d(-D / 2, -D / 2));
  static SwerveSimManager single = new SwerveSimManager();

  public static SwerveSimManager getInstance() {
    return single;
  }

  private Pose2d realPose = new Pose2d();
  private double[] thrustPos = {0, 0, 0, 0};
  private double[] lastthrustPos = {0, 0, 0, 0};
  private double[] thrustVel = {0, 0, 0, 0};
  private SlewRateLimiter[] thrustRateLimiters = {
    new SlewRateLimiter(3), new SlewRateLimiter(3), new SlewRateLimiter(3), new SlewRateLimiter(3)
  };
  private TrapezoidProfile.State[] steerStates = {
    new TrapezoidProfile.State(),
    new TrapezoidProfile.State(),
    new TrapezoidProfile.State(),
    new TrapezoidProfile.State()
  };
  private double[] cmdSteerPos = {0, 0, 0, 0};

  public SwerveSimManager() {}

  public void commandState(int mnum, SwerveModuleState state) {
    thrustVel[mnum] = thrustRateLimiters[mnum].calculate(state.speedMetersPerSecond);
    cmdSteerPos[mnum] = state.angle.getRotations();
  }

  public SwerveModulePosition getModPos(int mnum) {
    return new SwerveModulePosition(
        thrustPos[mnum], Rotation2d.fromRotations(steerStates[mnum].position));
  }

  public Rotation2d getIMUOutput() {
    return realPose.getRotation();
  }

  public void propagate() {
    for (int mnum = 0; mnum < 4; mnum++) {
      steerStates[mnum] =
          (new TrapezoidProfile(
                  new TrapezoidProfile.Constraints(6, 6),
                  new TrapezoidProfile.State(cmdSteerPos[mnum], 0),
                  steerStates[mnum]))
              .calculate(0.02);
      lastthrustPos = thrustPos.clone();
      thrustPos[mnum] += thrustVel[mnum] * 0.02;
    }
    realPose =
        realPose.exp(
            KINEMATICS.toTwist2d(
                new SwerveModulePosition(thrustVel[0] * 0.02, getModPos(0).angle),
                new SwerveModulePosition(thrustVel[1] * 0.02, getModPos(1).angle),
                new SwerveModulePosition(thrustVel[2] * 0.02, getModPos(2).angle),
                new SwerveModulePosition(thrustVel[3] * 0.02, getModPos(3).angle)));
  }
}
