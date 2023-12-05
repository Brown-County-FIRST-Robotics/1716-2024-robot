package frc.robot.subsystems.mecanum;


import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMUIO;
import frc.robot.subsystems.IMUIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class MecanumDrivetrain implements Drivetrain {
  public static final MecanumDriveKinematics KINEMATICS=new MecanumDriveKinematics(
          new Translation2d( 25.75*0.0254/2,  18.75*0.0254/2),
          new Translation2d( 25.75*0.0254/2, -18.75*0.0254/2),
          new Translation2d(-25.75*0.0254/2,  18.75*0.0254/2),
          new Translation2d(-25.75*0.0254/2, -18.75*0.0254/2));
  MecanumIO drive;
  IMUIO imu;
  MecanumIOInputsAutoLogged driveInputs=new MecanumIOInputsAutoLogged();
  IMUIOInputsAutoLogged imuInputs=new IMUIOInputsAutoLogged();
  MecanumDrivePoseEstimator poseEstimator;

  public MecanumDrivetrain(MecanumIO drive, IMUIO imu) {
    this.drive = drive;
    this.imu = imu;
    drive.updateInputs(driveInputs);
    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/MecanumInputs", driveInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
    poseEstimator=new MecanumDrivePoseEstimator(KINEMATICS, Rotation2d.fromDegrees(imuInputs.yaw),driveInputs.pos, Constants.INIT_POSE);
  }

  @Override
  public void periodic(){
    drive.updateInputs(driveInputs);
    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/MecanumInputs", driveInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
    poseEstimator.update(Rotation2d.fromDegrees(imuInputs.yaw),driveInputs.pos);
  }
  @Override
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void setPosition(Pose2d newPose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(imuInputs.yaw),driveInputs.pos,newPose);
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose) {
    return null;
  }

  @Override
  public Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY) {
    return null;
  }

  @Override
  public Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose) {
    return null;
  }

  @Override
  public Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY) {
    return null;
  }

  @Override
  public void humanDrive(ChassisSpeeds cmd, boolean foc) {
    ChassisSpeeds sp=new ChassisSpeeds(-cmd.vxMetersPerSecond, -cmd.vyMetersPerSecond, -cmd.omegaRadiansPerSecond);
    if (foc) {
      sp = ChassisSpeeds.fromFieldRelativeSpeeds(sp, getPosition().getRotation().unaryMinus());
    }
    MecanumDriveWheelSpeeds speeds = KINEMATICS.toWheelSpeeds(sp);
    drive.setSpeeds(speeds);
  }

  @Override
  public Rotation3d getGyro() {
    return new Rotation3d(imuInputs.roll,imuInputs.pitch,imuInputs.yaw);
  }

  @Override
  public double[] getAcceleration() {
    return new double[]{imuInputs.xAccelMPS, imuInputs.yAccelMPS, imuInputs.zAccelMPS};
  }
}
