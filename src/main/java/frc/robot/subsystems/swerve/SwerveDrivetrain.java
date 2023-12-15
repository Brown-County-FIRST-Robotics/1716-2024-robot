package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SuppliedCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMUIO;
import frc.robot.subsystems.IMUIOInputsAutoLogged;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrain implements Drivetrain {
  public static final double D = 21.125 * 0.0254; // TODO: Rename this
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(D / 2, D / 2),
          new Translation2d(D / 2, -D / 2),
          new Translation2d(-D / 2, D / 2),
          new Translation2d(-D / 2, -D / 2));
  public static final double MAX_WHEEL_SPEED = 5.0;
  ModuleIO fl;
  ModuleIO fr;
  ModuleIO bl;
  ModuleIO br;
  ModuleIOInputsAutoLogged flInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged blInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged brInputs = new ModuleIOInputsAutoLogged();

  IMUIO imu;
  IMUIOInputsAutoLogged imuInputs = new IMUIOInputsAutoLogged();

  SwerveDrivePoseEstimator poseEstimator;
  Field2d field;

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(flInputs.thrustPos, Rotation2d.fromRotations(flInputs.steerPos)),
      new SwerveModulePosition(frInputs.thrustPos, Rotation2d.fromRotations(frInputs.steerPos)),
      new SwerveModulePosition(blInputs.thrustPos, Rotation2d.fromRotations(blInputs.steerPos)),
      new SwerveModulePosition(brInputs.thrustPos, Rotation2d.fromRotations(brInputs.steerPos))
    };
  }

  public SwerveDrivetrain(ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br, IMUIO imu) {
    this.imu = imu;
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
    field = new Field2d();
    fl.updateInputs(flInputs);
    fr.updateInputs(frInputs);
    bl.updateInputs(blInputs);
    br.updateInputs(brInputs);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            KINEMATICS, getNavxRotation(), getPositions(), Constants.INIT_POSE);
  }

  @Override
  public void periodic() {
    imu.updateInputs(imuInputs);
    Logger.getInstance().processInputs("Drive/IMU", imuInputs);
    fl.updateInputs(flInputs);
    fr.updateInputs(frInputs);
    bl.updateInputs(blInputs);
    br.updateInputs(brInputs);
    Logger.getInstance().processInputs("Drive/FL", flInputs);
    Logger.getInstance().processInputs("Drive/FR", frInputs);
    Logger.getInstance().processInputs("Drive/BL", blInputs);
    Logger.getInstance().processInputs("Drive/BR", brInputs);

    poseEstimator.update(getNavxRotation(), getPositions());
    Logger.getInstance().recordOutput("Drive/Pose", getPosition());
    field.setRobotPose(getPosition());
  }

  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getNavxRotation() {
    return Rotation2d.fromDegrees(imuInputs.yaw);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED);
    states[0] =
        SwerveModuleState.optimize(
            states[0], getPositions()[0].angle.plus(Rotation2d.fromRotations(1.0)));
    states[1] =
        SwerveModuleState.optimize(
            states[1], getPositions()[1].angle.plus(Rotation2d.fromRotations(1.0)));
    states[2] =
        SwerveModuleState.optimize(
            states[2], getPositions()[2].angle.plus(Rotation2d.fromRotations(1.0)));
    states[3] =
        SwerveModuleState.optimize(
            states[3], getPositions()[3].angle.plus(Rotation2d.fromRotations(1.0)));
    Logger.getInstance().recordOutput("Drive/CmdStates", states);
    fl.setCmdState(states[0]);
    fr.setCmdState(states[1]);
    bl.setCmdState(states[2]);
    br.setCmdState(states[3]);
  }

  public Command makeTrajectoryCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(
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
        this::setModuleStates,
        this);
  }

  public void setPosition(Pose2d pos) {
    poseEstimator.resetPosition(getNavxRotation(), getPositions(), pos);
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose) {
    return getDriveToPointCmd(pose, 0, 0);
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY) {
    return new SuppliedCommand(
        () -> {
          TrajectoryConfig conf =
              new TrajectoryConfig(Constants.Auto.MAX_VELOCITY, Constants.Auto.MAX_ACCELERATION)
                  .setEndVelocity(Math.hypot(endVelX, endVelY));
          conf.setKinematics(KINEMATICS);
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(getPosition(), List.of(), pose, conf);
          return makeTrajectoryCommand(trajectory);
        },
        this);
  }

  @Override
  public Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose) {
    return getFollowWaypointsCmd(waypoints, pose, 0, 0);
  }

  @Override
  public Command getFollowWaypointsCmd(
      List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY) {
    return new SuppliedCommand(
        () -> {
          TrajectoryConfig conf =
              new TrajectoryConfig(Constants.Auto.MAX_VELOCITY, Constants.Auto.MAX_ACCELERATION)
                  .setEndVelocity(Math.hypot(endVelX, endVelY));
          conf.setKinematics(KINEMATICS);
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(getPosition(), waypoints, pose, conf);
          return makeTrajectoryCommand(trajectory);
        },
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
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(sp);
    setModuleStates(states);
  }

  @Override
  public Rotation3d getGyro() {
    return new Rotation3d(imuInputs.roll, imuInputs.pitch, imuInputs.yaw);
  }

  @Override
  public double[] getAcceleration() {
    return new double[] {imuInputs.xAccelMPS, imuInputs.yAccelMPS, imuInputs.zAccelMPS};
  }

  @Override
  public void lockWheels() {
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
  }
}
