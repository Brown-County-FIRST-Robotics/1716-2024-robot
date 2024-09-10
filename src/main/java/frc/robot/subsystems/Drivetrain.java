package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.PoseEstimator;

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
   * @param stdDevs The measurement error standard deviations [x, y, theta] [meters,meters,radians]
   * @param timestamp The time at which the pose was detected
   */
  void addVisionUpdate(Pose2d newPose, Vector<N3> stdDevs, double timestamp);

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

  PoseEstimator getPE();

  /** Locks the wheels. In mecanum, this does nothing. */
  default void lockWheels() {}

  /**
   * Gets the velocity according to the wheels (includes slip error)
   *
   * @return The velocity
   */
  ChassisSpeeds getVelocity();
}
