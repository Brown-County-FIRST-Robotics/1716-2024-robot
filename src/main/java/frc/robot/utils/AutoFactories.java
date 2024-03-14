package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.RotateTo;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import java.util.List;
import java.util.Optional;

public class AutoFactories {
  private static Trajectory makeTrajectory(Drivetrain drive, Pose2d target) {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(5, 10);
    Rotation2d realAng;
    if (ShootWhileMove.getFieldRelativeSpeeds(
                drive.getVelocity(), drive.getPosition().getRotation())
            .getNorm()
        < 0.1) {
      realAng = target.getTranslation().minus(drive.getPosition().getTranslation()).getAngle();
    } else {
      var speed =
          ShootWhileMove.getFieldRelativeSpeeds(
              drive.getVelocity(), drive.getPosition().getRotation());
      trajectoryConfig.setEndVelocity(speed.getNorm());
      realAng = speed.getAngle();
    }
    return TrajectoryGenerator.generateTrajectory(
        new Pose2d(drive.getPosition().getTranslation(), realAng),
        List.of(),
        target,
        trajectoryConfig);
  }

  private static Trajectory makeTrajectory(Drivetrain drive, Translation2d target) {
    return makeTrajectory(
        drive, new Pose2d(target, drive.getPosition().getTranslation().minus(target).getAngle()));
  }

  public static Command speaker(Drivetrain drivetrain, Arm arm, Shooter shooter) {
    RotateTo rt = new RotateTo(drivetrain);
    SpeakerShoot speakerShoot = new SpeakerShoot(drivetrain, arm, rt::setCustomRotation, shooter);
    return speakerShoot.raceWith(rt.repeatedly());
  }

  public static Command pickup(Drivetrain drivetrain, Arm arm, Shooter shooter, int pos) {
    Translation2d target = FieldConstants.getGamePiece(pos);
    HolonomicTrajectoryFollower trajectoryCommand =
        new HolonomicTrajectoryFollower(drivetrain, () -> makeTrajectory(drivetrain, target));

    return Intake.fromFloor(shooter, arm)
        .raceWith(
            trajectoryCommand
                .alongWith(
                    Commands.run(
                        () ->
                            trajectoryCommand.setCustomRotation(
                                Optional.of(
                                    drivetrain
                                        .getPosition()
                                        .getTranslation()
                                        .minus(target)
                                        .getAngle()))))
                .andThen(Commands.waitSeconds(1)));
  }

  public static Command pickupWithBackup(
      Drivetrain drivetrain, Arm arm, Shooter shooter, int pos, int backupPos) {
    return pickup(drivetrain, arm, shooter, pos)
        .andThen(
            Commands.either(
                Commands.none(), pickup(drivetrain, arm, shooter, backupPos), shooter::isHolding));
  }
}
