package frc.robot.subsystems.swerve;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.utils.Overrides;
import frc.robot.utils.PoseEstimator;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** The swerve drivetrain subsystem */
public class SwerveDrivetrain implements Drivetrain {
  private static final double D = 21.125 * 0.0254; // TODO: Rename this
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(D / 2, D / 2),
          new Translation2d(D / 2, -D / 2),
          new Translation2d(-D / 2, D / 2),
          new Translation2d(-D / 2, -D / 2));
  private static final double MAX_WHEEL_SPEED = 5.0;
  ModuleIO fl;
  ModuleIO fr;
  ModuleIO bl;
  ModuleIO br;
  ModuleIOInputsAutoLogged flInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged frInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged blInputs = new ModuleIOInputsAutoLogged();
  ModuleIOInputsAutoLogged brInputs = new ModuleIOInputsAutoLogged();
  Rotation2d lastIMU;
  SwerveModulePosition[] lastPositions;
  PoseEstimator poseEstimator;

  IMUIO imu;
  IMUIOInputsAutoLogged imuInputs = new IMUIOInputsAutoLogged();

  private SwerveModulePosition[] getPositions() {

    return new SwerveModulePosition[] {
      new SwerveModulePosition(
          flInputs.pos.distanceMeters,
          flInputs
              .pos
              .angle
              .minus(Rotation2d.fromRotations(flInputs.offset))
              .unaryMinus()
              .minus(Rotation2d.fromDegrees(-90))),
      new SwerveModulePosition(
          frInputs.pos.distanceMeters,
          frInputs
              .pos
              .angle
              .minus(Rotation2d.fromRotations(frInputs.offset))
              .unaryMinus()
              .minus(Rotation2d.fromDegrees(0))),
      new SwerveModulePosition(
          blInputs.pos.distanceMeters,
          blInputs
              .pos
              .angle
              .minus(Rotation2d.fromRotations(blInputs.offset))
              .unaryMinus()
              .minus(Rotation2d.fromDegrees(180))),
      new SwerveModulePosition(
          brInputs.pos.distanceMeters,
          brInputs
              .pos
              .angle
              .minus(Rotation2d.fromRotations(brInputs.offset))
              .unaryMinus()
              .minus(Rotation2d.fromDegrees(90)))
    };
  }

  /**
   * Creates a SwerveDrivetrain from IO
   *
   * @param fl Front left module IO
   * @param fr Front right module IO
   * @param bl Back left module IO
   * @param br Back right module IO
   * @param imu IMU IO
   */
  public SwerveDrivetrain(ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br, IMUIO imu) {
    this.imu = imu;
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
    fl.updateInputs(flInputs);
    fr.updateInputs(frInputs);
    bl.updateInputs(blInputs);
    br.updateInputs(brInputs);
    poseEstimator = new PoseEstimator();
    poseEstimator.setPose(Constants.INIT_POSE);
    lastIMU = getGyro().toRotation2d();
    lastPositions = getPositions();
  }

  @Override
  public void periodic() {
    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
    fl.updateInputs(flInputs);
    fr.updateInputs(frInputs);
    bl.updateInputs(blInputs);
    br.updateInputs(brInputs);
    Logger.processInputs("Drive/FL", flInputs);
    Logger.processInputs("Drive/FR", frInputs);
    Logger.processInputs("Drive/BL", blInputs);
    Logger.processInputs("Drive/BR", brInputs);

    Logger.recordOutput("Drive/RealStates", getWheelSpeeds());
    Twist2d odoTwist =
        KINEMATICS.toTwist2d(
            new SwerveDriveWheelPositions(lastPositions),
            new SwerveDriveWheelPositions(getPositions()));
    if (!Overrides.disableIMU.get()) {
      odoTwist =
          new Twist2d(
              odoTwist.dx, odoTwist.dy, getGyro().toRotation2d().minus(lastIMU).getRadians());
    }
    poseEstimator.addOdometry(odoTwist);
    lastPositions = getPositions();
    lastIMU = getGyro().toRotation2d();
    Logger.recordOutput("Drive/Pose", getPosition());

    checkForYawReset();
  }

  private void checkForYawReset() {
    if (Overrides.resetYaw.get()) {
      poseEstimator.setPose(
          new Pose2d(getPosition().getTranslation(), Constants.INIT_POSE.getRotation()));
      Overrides.resetYaw.set(false);
    }
  }

  private SwerveModuleState[] getWheelSpeeds() {
    return new SwerveModuleState[] {
      new SwerveModuleState(flInputs.vel.speedMetersPerSecond, getPositions()[0].angle),
      new SwerveModuleState(frInputs.vel.speedMetersPerSecond, getPositions()[1].angle),
      new SwerveModuleState(blInputs.vel.speedMetersPerSecond, getPositions()[2].angle),
      new SwerveModuleState(brInputs.vel.speedMetersPerSecond, getPositions()[3].angle)
    };
  }

  @Override
  public Pose2d getPosition() {
    return poseEstimator.getPose();
  }

  private void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED);
    states[0] = SwerveModuleState.optimize(states[0], getPositions()[0].angle);
    states[1] = SwerveModuleState.optimize(states[1], getPositions()[1].angle);
    states[2] = SwerveModuleState.optimize(states[2], getPositions()[2].angle);
    states[3] = SwerveModuleState.optimize(states[3], getPositions()[3].angle);
    states[0].speedMetersPerSecond *= getPositions()[0].angle.minus(states[0].angle).getCos();
    states[1].speedMetersPerSecond *= getPositions()[1].angle.minus(states[1].angle).getCos();
    states[2].speedMetersPerSecond *= getPositions()[2].angle.minus(states[2].angle).getCos();
    states[3].speedMetersPerSecond *= getPositions()[3].angle.minus(states[3].angle).getCos();

    Logger.recordOutput("Drive/CmdStates", states);
    fl.setCmdState(
        new SwerveModuleState(
            states[0].speedMetersPerSecond,
            states[0]
                .angle
                .plus(Rotation2d.fromDegrees(-90))
                .unaryMinus()
                .plus(Rotation2d.fromRotations(flInputs.offset))));
    fr.setCmdState(
        new SwerveModuleState(
            states[1].speedMetersPerSecond,
            states[1]
                .angle
                .plus(Rotation2d.fromDegrees(0))
                .unaryMinus()
                .plus(Rotation2d.fromRotations(frInputs.offset))));
    bl.setCmdState(
        new SwerveModuleState(
            states[2].speedMetersPerSecond,
            states[2]
                .angle
                .plus(Rotation2d.fromDegrees(180))
                .unaryMinus()
                .plus(Rotation2d.fromRotations(blInputs.offset))));
    br.setCmdState(
        new SwerveModuleState(
            states[3].speedMetersPerSecond,
            states[3]
                .angle
                .plus(Rotation2d.fromDegrees(90))
                .unaryMinus()
                .plus(Rotation2d.fromRotations(brInputs.offset))));
  }

  private Command makeTrajectoryCommand(Trajectory trajectory) {
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

  @Override
  public void setPosition(Pose2d pos) {
    poseEstimator.setPose(pos);
  }

  @Override
  public void addVisionUpdate(Pose2d newPose, Vector<N3> stdDevs, double timestamp) {
    poseEstimator.addVision(newPose, stdDevs, timestamp);
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose) {
    return getDriveToPointCmd(pose, 0, 0);
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY) {
    return new DeferredCommand(
        () -> {
          TrajectoryConfig conf =
              new TrajectoryConfig(Constants.Auto.MAX_VELOCITY, Constants.Auto.MAX_ACCELERATION)
                  .setEndVelocity(Math.hypot(endVelX, endVelY));
          conf.setKinematics(KINEMATICS);
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(getPosition(), List.of(), pose, conf);
          return makeTrajectoryCommand(trajectory);
        },
        Set.of(this));
  }

  @Override
  public Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose) {
    return getFollowWaypointsCmd(waypoints, pose, 0, 0);
  }

  @Override
  public Command getFollowWaypointsCmd(
      List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY) {
    return new DeferredCommand(
        () -> {
          TrajectoryConfig conf =
              new TrajectoryConfig(Constants.Auto.MAX_VELOCITY, Constants.Auto.MAX_ACCELERATION)
                  .setEndVelocity(Math.hypot(endVelX, endVelY));
          conf.setKinematics(KINEMATICS);
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(getPosition(), waypoints, pose, conf);
          return makeTrajectoryCommand(trajectory);
        },
        (Set<Subsystem>) this);
  }

  @Override
  public void humanDrive(ChassisSpeeds cmd) {
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(cmd);
    setModuleStates(states);
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
  public void lockWheels() {
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
  }

  @Override
  public ChassisSpeeds getVelocity() {
    return KINEMATICS.toChassisSpeeds(getWheelSpeeds());
  }
}
