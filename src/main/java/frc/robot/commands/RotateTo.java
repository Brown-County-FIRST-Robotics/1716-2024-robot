package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.HolonomicTrajectoryFollower;
import java.util.Optional;

public class RotateTo extends Command {
  Drivetrain drivetrain;
  Optional<Rotation2d> customRotation = Optional.empty();

  public void setCustomRotation(Optional<Rotation2d> customRotation) {
    this.customRotation = customRotation;
  }

  public RotateTo(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  public RotateTo(Drivetrain drivetrain, Rotation2d customRotation) {
    this.drivetrain = drivetrain;
    this.customRotation = Optional.of(customRotation);
  }

  @Override
  public void execute() {
    drivetrain.humanDrive(
        new ChassisSpeeds(
            0,
            0,
            customRotation
                .map(
                    rotation2d ->
                        HolonomicTrajectoryFollower.getExt(
                            rotation2d,
                            drivetrain.getPosition().getRotation(),
                            drivetrain.getVelocity().omegaRadiansPerSecond))
                .orElse(0.0)));
  }

  @Override
  public boolean isFinished() {
    return customRotation
        .map(
            rotation2d ->
                (Math.abs(rotation2d.minus(drivetrain.getPosition().getRotation()).getDegrees())
                    < HolonomicTrajectoryFollower.allowedErr.get()))
        .orElse(true);
  }
}
