package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

/** This interface represents a holonomic drivetrain */
public interface Drivetrain extends Subsystem {
  /**
   * Gets the position from the pose estimator
   *
   * @return The position from the pose estimator
   */
  Pose2d getPosition();

  /**
   * Sets the position of the pose estimator
   *
   * @param newPose The new pose to use
   */
  void setPosition(Pose2d newPose);

  /**
   * Adds a vision update to the pose estimator
   *
   * @param newPose The estimated pose
   * @param timestamp The time at which the pose was detected
   */
  void addVisionUpdate(Pose2d newPose, double timestamp);
  /**
   * Returns a command that drives to a point
   *
   * @param pose The ending pose
   * @return The command that goes to the point
   */
  Command getDriveToPointCmd(Pose2d pose);
  /**
   * Returns a command that drives to a point
   *
   * @param pose The ending pose
   * @param endVelX The ending velocity in the x direction
   * @param endVelY The ending velocity in the y direction
   * @return The command that goes to the point
   */
  Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY);
  /**
   * Returns a command that follows the given waypoints
   *
   * @param waypoints The waypoints to intersect in the trajectory
   * @param pose The ending pose
   * @return The command that follows the waypoints
   */
  Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose);

  /**
   * Returns a command that follows the given waypoints
   *
   * @param waypoints The waypoints to intersect in the trajectory
   * @param pose The ending pose
   * @param endVelX The ending velocity in the x direction
   * @param endVelY The ending velocity in the y direction
   * @return The command that follows the waypoints
   */
  Command getFollowWaypointsCmd(
      List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY);

  /**
   * Commands a <code>ChassisSpeeds</code> to the drivetrain
   *
   * @param cmd The commanded speeds
   */
  void humanDrive(ChassisSpeeds cmd);

  /**
   * Gets the current orientation according to the gyro
   *
   * @return The value from the gyro
   */
  Rotation3d getGyro();
  /**
   * Gets the acceleration values from the IMU
   *
   * @return Array of accelerations (in MPS^2) as [x,y,z]
   */
  double[] getAcceleration();

  /** Locks the wheels. In mecanum, this does nothing. */
  default void lockWheels() {}

  /**
   * Gets the velocity according to the wheels (includes slip error)
   *
   * @return The velocity
   */
  ChassisSpeeds getVelocity();
}
