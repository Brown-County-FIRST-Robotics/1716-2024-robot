package frc.robot.subsystems.mecanum;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.Overrides;
import frc.robot.utils.PoseEstimator;
import org.littletonrobotics.junction.Logger;

/** The mecanum drivetrain subsystem */
public class MecanumDrivetrain implements Drivetrain {
  static final MecanumDriveKinematics KINEMATICS =
      new MecanumDriveKinematics(
          new Translation2d(25.75 * 0.0254 / 2, 18.75 * 0.0254 / 2),
          new Translation2d(25.75 * 0.0254 / 2, -18.75 * 0.0254 / 2),
          new Translation2d(-25.75 * 0.0254 / 2, 18.75 * 0.0254 / 2),
          new Translation2d(-25.75 * 0.0254 / 2, -18.75 * 0.0254 / 2));
  final MecanumIO drive;
  final IMUIO imu;
  final MecanumIOInputsAutoLogged driveInputs = new MecanumIOInputsAutoLogged();
  final IMUIOInputsAutoLogged imuInputs = new IMUIOInputsAutoLogged();
  Rotation2d lastIMU;
  MecanumDriveWheelPositions lastPositions;
  final PoseEstimator poseEstimator;
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
    CustomAlerts.makeOverTempAlert(() -> driveInputs.flTemp, 60, 50, "FL motor");
    CustomAlerts.makeOverTempAlert(() -> driveInputs.frTemp, 60, 50, "FR motor");
    CustomAlerts.makeOverTempAlert(() -> driveInputs.blTemp, 60, 50, "BL motor");
    CustomAlerts.makeOverTempAlert(() -> driveInputs.brTemp, 60, 50, "BR motor");
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
    if (!Overrides.disableIMU.get()) {
      odoTwist =
          new Twist2d(
              odoTwist.dx, odoTwist.dy, getGyro().toRotation2d().minus(lastIMU).getRadians());
    }
    poseEstimator.addOdometry(odoTwist);
    lastIMU = getGyro().toRotation2d();
    lastPositions = driveInputs.pos;

    checkForYawReset();
  }

  private void checkForYawReset() {
    if (Overrides.resetYaw.get()) {
      poseEstimator.setPose(
          new Pose2d(getPosition().getTranslation(), Constants.INIT_POSE.getRotation()));
      Overrides.resetYaw.set(false);
    }
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
  public PoseEstimator getPE() {
    return poseEstimator;
  }
}
