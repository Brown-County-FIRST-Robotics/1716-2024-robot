package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HolonomicTrajectoryFollower extends Command {
  private static final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(3, 20);
  private static LoggedTunableNumber allowedErr =
      new LoggedTunableNumber("Rotation Allowed Err", 3);
  private static LoggedTunableNumber replanErr =
      new LoggedTunableNumber("Replanning threshold", 0.1);

  public static double getExt(
      Rotation2d cmdRotation, Rotation2d currentRotation, double currentVelocity) {
    if (Math.abs(cmdRotation.minus(currentRotation).getDegrees()) > allowedErr.get()) {
      return new TrapezoidProfile(constraints)
          .calculate(
              0.02,
              new TrapezoidProfile.State(currentRotation.getRadians(), currentVelocity),
              new TrapezoidProfile.State(cmdRotation.getRadians(), 0))
          .velocity;
    }
    return 0;
  }

  Drivetrain drivetrain;
  Supplier<Trajectory> trajectorySupplier;
  Trajectory activeTrajectory;
  Timer timer = new Timer();
  Optional<Rotation2d> customRotation = Optional.empty();

  public void setCustomRotation(Optional<Rotation2d> customRotation) {
    this.customRotation = customRotation;
  }

  public HolonomicTrajectoryFollower(
      Drivetrain drive, Supplier<Trajectory> trajectorySupplier, Rotation2d targetRotation) {
    this(drive, trajectorySupplier);
    setCustomRotation(Optional.of(targetRotation));
  }

  public HolonomicTrajectoryFollower(Drivetrain drive, Supplier<Trajectory> trajectorySupplier) {
    this.drivetrain = drive;
    this.trajectorySupplier = trajectorySupplier;
    activeTrajectory = trajectorySupplier.get();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (activeTrajectory
            .sample(timer.get())
            .poseMeters
            .getTranslation()
            .getDistance(drivetrain.getPosition().getTranslation())
        > replanErr.get()) {
      activeTrajectory = trajectorySupplier.get();
      timer.reset();
      timer.start();
    }
    var state = activeTrajectory.sample(timer.get());
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            state.velocityMetersPerSecond * state.poseMeters.getRotation().getCos(),
            state.velocityMetersPerSecond * state.poseMeters.getRotation().getSin(),
            customRotation
                .map(
                    rotation2d ->
                        getExt(
                            rotation2d,
                            drivetrain.getPosition().getRotation(),
                            drivetrain.getVelocity().omegaRadiansPerSecond))
                .orElse(0.0));
    var discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    drivetrain.humanDrive(discreteSpeeds);
    Logger.recordOutput("Follower/CurrentTrajectory", activeTrajectory);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(activeTrajectory.getTotalTimeSeconds())
        && (customRotation
            .filter(
                rotation2d ->
                    (Math.abs(rotation2d.minus(drivetrain.getPosition().getRotation()).getDegrees())
                        > allowedErr.get()))
            .isPresent());
  }

  public Translation2d getPoseAtTime(double t) {
    return activeTrajectory.sample(timer.get() + t).poseMeters.getTranslation();
  }

  public Translation2d getSpeedAtTime(double t) {
    return new Translation2d(
        activeTrajectory.sample(timer.get() + t).velocityMetersPerSecond,
        activeTrajectory.sample(timer.get() + t).poseMeters.getRotation());
  }
}
