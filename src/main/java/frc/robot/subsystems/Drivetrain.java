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

  void addVisionUpdate(Pose2d newPose, double timestamp);

  Command getDriveToPointCmd(Pose2d pose);

  Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY);

  Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose);

  Command getFollowWaypointsCmd(
      List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY);

  void humanDrive(ChassisSpeeds cmd, boolean foc);

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
