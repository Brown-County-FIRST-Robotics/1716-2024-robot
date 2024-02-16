package frc.robot.subsystems.mecanum;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.utils.PoseEstimator;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** The mecanum drivetrain subsystem */
public class MecanumDrivetrain implements Drivetrain {
  static final double MAX_WHEEL_SPEED = 5.85;
  static final MecanumDriveKinematics KINEMATICS =
      new MecanumDriveKinematics(
          new Translation2d(25.75 * 0.0254 / 2, 18.75 * 0.0254 / 2),
          new Translation2d(25.75 * 0.0254 / 2, -18.75 * 0.0254 / 2),
          new Translation2d(-25.75 * 0.0254 / 2, 18.75 * 0.0254 / 2),
          new Translation2d(-25.75 * 0.0254 / 2, -18.75 * 0.0254 / 2));
  MecanumIO drive;
  IMUIO imu;
  MecanumIOInputsAutoLogged driveInputs = new MecanumIOInputsAutoLogged();
  IMUIOInputsAutoLogged imuInputs = new IMUIOInputsAutoLogged();
  Rotation2d lastIMU;
  MecanumDriveWheelPositions lastPositions;
  PoseEstimator poseEstimator;
  /**
   * Constructs a <code>MecanumDrivetrain</code> from IO
   *
   * @param drive Drive IO
   * @param imu Imu IO
   */
  public MecanumDrivetrain(MecanumIO drive, IMUIO imu) {
    this.drive = drive;
    this.imu = imu;
    drive.updateInputs(driveInputs);
    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/MecanumInputs", driveInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
    poseEstimator = new PoseEstimator();
    poseEstimator.setPose(Constants.INIT_POSE);
    lastIMU = getGyro().toRotation2d();
    lastPositions = driveInputs.pos;
  }

  @Override
  public void periodic() {
    drive.updateInputs(driveInputs);
    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/MecanumInputs", driveInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
    Logger.recordOutput("Drive/Pose", getPosition());
    Logger.recordOutput(
        "Drive/RealSpeeds",
        new SwerveModuleState(
            driveInputs.vel.frontLeftMetersPerSecond, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(
            driveInputs.vel.frontRightMetersPerSecond, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(driveInputs.vel.rearLeftMetersPerSecond, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(
            driveInputs.vel.rearRightMetersPerSecond, Rotation2d.fromDegrees(-45)));
    Twist2d odoTwist = KINEMATICS.toTwist2d(driveInputs.pos, lastPositions);
    odoTwist =
        new Twist2d(odoTwist.dx, odoTwist.dy, getGyro().toRotation2d().minus(lastIMU).getRadians());
    poseEstimator.addOdometry(odoTwist);
    lastIMU = getGyro().toRotation2d();
    lastPositions = driveInputs.pos;
  }

  @Override
  public Pose2d getPosition() {
    return poseEstimator.getPose();
  }

  @Override
  public void setPosition(Pose2d newPose) {
    poseEstimator.setPose(newPose);
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose) {
    return null;
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY) {
    return new DeferredCommand(
        () -> {
          TrajectoryConfig conf =
              new TrajectoryConfig(Constants.Auto.MAX_VELOCITY, Constants.Auto.MAX_ACCELERATION)
                  .setEndVelocity(Math.hypot(endVelX, endVelY));
          // conf.setKinematics(KINEMATICS);
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(getPosition(), List.of(), pose, conf);
          return makeTrajectoryCommand(trajectory);
        },
        (Set<Subsystem>) this);
  }

  @Override
  public Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose) {
    return null;
  }

  @Override
  public Command getFollowWaypointsCmd(
      List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY) {
    return null;
  }

  Command makeTrajectoryCommand(Trajectory trajectory) {
    Logger.recordOutput("Drive/CurrentTraj", trajectory);
    return new MecanumControllerCommand(
        trajectory,
        this::getPosition,
        KINEMATICS,
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        new ProfiledPIDController(
            0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.Auto.MAX_ANGULAR_VELOCITY, Constants.Auto.MAX_ANGULAR_ACCELERATION)),
        MAX_WHEEL_SPEED,
        this::setWheelSpeeds,
        this);
  }

  @Override
  public void humanDrive(ChassisSpeeds cmd) {
    MecanumDriveWheelSpeeds speeds = KINEMATICS.toWheelSpeeds(cmd);
    setWheelSpeeds(speeds);
  }

  private void setWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
    Logger.recordOutput(
        "Drive/CmdSpeeds",
        new SwerveModuleState(speeds.frontLeftMetersPerSecond, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(speeds.frontRightMetersPerSecond, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(speeds.rearLeftMetersPerSecond, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(speeds.rearRightMetersPerSecond, Rotation2d.fromDegrees(-45)));
    drive.setSpeeds(speeds);
  }

  @Override
  public Rotation3d getGyro() {
    return imuInputs.rotation;
  }

  @Override
  public double[] getAcceleration() {
    return new double[] {imuInputs.xAccelMPS, imuInputs.yAccelMPS, imuInputs.zAccelMPS};
  }

  @Override
  public ChassisSpeeds getVelocity() {
    return KINEMATICS.toChassisSpeeds(driveInputs.vel);
  }

  @Override
  public void addVisionUpdate(Pose2d newPose, Vector<N3> stdDevs, double timestamp) {
    poseEstimator.addVision(newPose, stdDevs, timestamp);
  }

  @Override
  public void addVisionUpdate(Pose2d newPose, double timestamp, int tags) {}
}
