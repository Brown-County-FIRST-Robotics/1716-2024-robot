package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.DualRateLimiter;
import org.littletonrobotics.junction.Logger;

/**
 * Manages swerve simulation <br>
 * Access using {@link #getInstance()}
 */
public class SwerveSimManager {
  private static final double D = 21.125 * 0.0254; // TODO: Rename this
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(D / 2, D / 2),
          new Translation2d(D / 2, -D / 2),
          new Translation2d(-D / 2, D / 2),
          new Translation2d(-D / 2, -D / 2));
  static SwerveSimManager single = new SwerveSimManager();

  /**
   * Gets the global instance of SwerveSim
   *
   * @return The global instance
   */
  public static SwerveSimManager getInstance() {
    return single;
  }

  private Pose2d realPose = new Pose2d();
  private double[] thrustPos = {0, 0, 0, 0};
  private double[] lastthrustPos = {0, 0, 0, 0};
  private double[] thrustVel = {0, 0, 0, 0};
  private final double thrustMaxAccel = 5;
  private final double thrustBrakeAccel = 30;
  private DualRateLimiter[] thrustRateLimiters = {
    new DualRateLimiter(thrustMaxAccel, thrustBrakeAccel),
    new DualRateLimiter(thrustMaxAccel, thrustBrakeAccel),
    new DualRateLimiter(thrustMaxAccel, thrustBrakeAccel),
    new DualRateLimiter(thrustMaxAccel, thrustBrakeAccel)
  };
  private TrapezoidProfile.State[] steerStates = {
    new TrapezoidProfile.State(),
    new TrapezoidProfile.State(),
    new TrapezoidProfile.State(),
    new TrapezoidProfile.State()
  };
  private double[] cmdSteerPos = {0, 0, 0, 0};

  /** Constructs a new SwerveSimManager */
  public SwerveSimManager() {}

  /**
   * Sets the desired state of a swerve module
   *
   * @param mnum The index of the swerve module
   * @param state The desired state
   */
  public void commandState(int mnum, SwerveModuleState state) {
    thrustVel[mnum] = thrustRateLimiters[mnum].calculate(state.speedMetersPerSecond);
    cmdSteerPos[mnum] = state.angle.getRotations();
  }

  /**
   * Gets the position of a swerve module
   *
   * @param mnum The index of the module
   * @return The position of the module
   */
  public SwerveModulePosition getModPos(int mnum) {
    return new SwerveModulePosition(
        thrustPos[mnum], Rotation2d.fromRotations(steerStates[mnum].position));
  }

  /**
   * Gets the velocity of the steering motor of the swerve module
   *
   * @param mnum The index of the module
   * @return The steer motor velocity
   */
  public double getSteerVel(int mnum) {
    return steerStates[mnum].velocity;
  }

  /**
   * Gets the state of the swerve module
   *
   * @param mnum The index of the module
   * @return The state of the module
   */
  public SwerveModuleState getModState(int mnum) {
    return new SwerveModuleState(
        thrustVel[mnum], Rotation2d.fromRotations(steerStates[mnum].position));
  }

  /**
   * Gets the yaw value from the simulated IMU
   *
   * @return The yaw value
   */
  public Rotation3d getIMUOutput() {
    return new Rotation3d(0, 0, realPose.getRotation().getRadians());
  }

  /**
   * Step the simulation one tick (20 ms) forward. During simulation, this should be called at the
   * end of every loop.
   */
  public void propagate() {
    for (int mnum = 0; mnum < 4; mnum++) {
      if (Math.abs(
              Rotation2d.fromRotations(cmdSteerPos[mnum] - 1.0)
                  .minus(Rotation2d.fromRotations(steerStates[mnum].position))
                  .getRotations())
          <= 0.5) {
        cmdSteerPos[mnum] -= 1.0;
      } else if (Math.abs(
              Rotation2d.fromRotations(cmdSteerPos[mnum] + 1.0)
                  .minus(Rotation2d.fromRotations(steerStates[mnum].position))
                  .getRotations())
          <= 0.5) {
        cmdSteerPos[mnum] += 1.0;
      }
      steerStates[mnum] =
          (new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 20)))
              .calculate(0.02, new TrapezoidProfile.State(cmdSteerPos[mnum], 0), steerStates[mnum]);
      steerStates[mnum].position = ((steerStates[mnum].position % 1.0) + 1.0) % 1.0;
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
    Logger.recordOutput("Sim/RealPose", realPose);
  }
}
