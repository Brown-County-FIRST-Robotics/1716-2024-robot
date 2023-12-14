package frc.robot.subsystems.mecanum;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants;
import frc.robot.SuppliedCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMUIO;
import frc.robot.subsystems.IMUIOInputsAutoLogged;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class MecanumDrivetrain implements Drivetrain {
  public static final double MAX_WHEEL_SPEED = 5.85;
  public static final MecanumDriveKinematics KINEMATICS =
      new MecanumDriveKinematics(
          new Translation2d(25.75 * 0.0254 / 2, 18.75 * 0.0254 / 2),
          new Translation2d(25.75 * 0.0254 / 2, -18.75 * 0.0254 / 2),
          new Translation2d(-25.75 * 0.0254 / 2, 18.75 * 0.0254 / 2),
          new Translation2d(-25.75 * 0.0254 / 2, -18.75 * 0.0254 / 2));
  MecanumIO drive;
  IMUIO imu;
  MecanumIOInputsAutoLogged driveInputs = new MecanumIOInputsAutoLogged();
  IMUIOInputsAutoLogged imuInputs = new IMUIOInputsAutoLogged();
  MecanumDrivePoseEstimator poseEstimator;

  public MecanumDrivetrain(MecanumIO drive, IMUIO imu) {
    this.drive = drive;
    this.imu = imu;
    drive.updateInputs(driveInputs);
    imu.updateInputs(imuInputs);
    Logger.getInstance().processInputs("Drive/MecanumInputs", driveInputs);
    Logger.getInstance().processInputs("Drive/IMU", imuInputs);
    poseEstimator =
        new MecanumDrivePoseEstimator(
            KINEMATICS,
            Rotation2d.fromDegrees(imuInputs.yaw),
            new MecanumDriveWheelPositions(
                driveInputs.flPos, driveInputs.frPos, driveInputs.blPos, driveInputs.brPos),
            Constants.INIT_POSE);
  }

  @Override
  public void periodic() {
    drive.updateInputs(driveInputs);
    imu.updateInputs(imuInputs);
    Logger.getInstance().processInputs("Drive/MecanumInputs", driveInputs);
    Logger.getInstance().processInputs("Drive/IMU", imuInputs);
    Logger.getInstance().recordOutput("Drive/Pose", getPosition());
    poseEstimator.update(
        Rotation2d.fromDegrees(imuInputs.yaw),
        new MecanumDriveWheelPositions(
            driveInputs.flPos, driveInputs.frPos, driveInputs.blPos, driveInputs.brPos));
  }

  @Override
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void setPosition(Pose2d newPose) {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(imuInputs.yaw),
        new MecanumDriveWheelPositions(
            driveInputs.flPos, driveInputs.frPos, driveInputs.blPos, driveInputs.brPos),
        newPose);
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose) {
    return null;
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY) {
    return new SuppliedCommand(
        () -> {
          TrajectoryConfig conf =
              new TrajectoryConfig(Constants.Auto.MAX_VELOCITY, Constants.Auto.MAX_ACCELERATION)
                  .setEndVelocity(Math.hypot(endVelX, endVelY));
          // conf.setKinematics(KINEMATICS);
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(getPosition(), List.of(), pose, conf);
          return makeTrajectoryCommand(trajectory);
        },
        this);
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

  public Command makeTrajectoryCommand(Trajectory trajectory) {
    Logger.getInstance().recordOutput("Drive/CurrentTraj", trajectory);
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
  public void humanDrive(ChassisSpeeds cmd, boolean foc) {
    ChassisSpeeds sp =
        new ChassisSpeeds(
            -cmd.vxMetersPerSecond, -cmd.vyMetersPerSecond, -cmd.omegaRadiansPerSecond);
    if (foc) {
      Rotation2d rot =
          DriverStation.getAlliance() == DriverStation.Alliance.Red
              ? getPosition().getRotation()
              : getPosition().getRotation().rotateBy(Rotation2d.fromRotations(0.5));
      sp =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              new ChassisSpeeds(
                  cmd.vxMetersPerSecond, cmd.vyMetersPerSecond, -cmd.omegaRadiansPerSecond),
              rot);
    }
    MecanumDriveWheelSpeeds speeds = KINEMATICS.toWheelSpeeds(sp);
    setWheelSpeeds(speeds);
  }

  private void setWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
    Logger.getInstance()
        .recordOutput(
            "Drive/WheelSpeeds",
            new SwerveModuleState(speeds.frontLeftMetersPerSecond, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(speeds.frontRightMetersPerSecond, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(speeds.rearLeftMetersPerSecond, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(speeds.rearRightMetersPerSecond, Rotation2d.fromDegrees(-45)));
    drive.setSpeeds(speeds);
  }

  @Override
  public Rotation3d getGyro() {
    return new Rotation3d(imuInputs.roll, imuInputs.pitch, imuInputs.yaw);
  }

  @Override
  public double[] getAcceleration() {
    return new double[] {imuInputs.xAccelMPS, imuInputs.yAccelMPS, imuInputs.zAccelMPS};
  }
}
