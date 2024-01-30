package frc.robot.utils;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HolonomicTrajectoryFollower extends Command {
  Consumer<ChassisSpeeds> speedsConsumer;
  Trajectory trajectory;
  Timer timer;
  Supplier<Rotation2d> rotationSupplier;
  Supplier<Pose2d> realPoseSupplier;
  HolonomicDriveController controller;

  public HolonomicTrajectoryFollower(
      Consumer<ChassisSpeeds> speedsConsumer,
      Trajectory trajectory,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Pose2d> realPoseSupplier,
      HolonomicDriveController controller,
      Subsystem... requirements) {
    this.speedsConsumer = speedsConsumer;
    this.trajectory = trajectory;
    this.rotationSupplier = rotationSupplier;
    this.realPoseSupplier = realPoseSupplier;
    this.controller = controller;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Logger.recordOutput("Follower/CurrentTrajectory", trajectory);
  }

  @Override
  public void execute() {
    var state = trajectory.sample(timer.get());
    var speeds = controller.calculate(realPoseSupplier.get(), state, rotationSupplier.get());
    var discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    speedsConsumer.accept(discreteSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  public Translation2d getPoseAtTime(double t) {
    return trajectory.sample(timer.get() + t).poseMeters.getTranslation();
  }

  public Translation2d getSpeedAtTime(double t) {
    return new Translation2d(
        trajectory.sample(timer.get() + t).velocityMetersPerSecond,
        trajectory.sample(timer.get() + t).poseMeters.getRotation());
  }
}
