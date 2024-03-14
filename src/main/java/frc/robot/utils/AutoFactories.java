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

/** A bunch of static factory methods for creating auto routines */
public class AutoFactories {
  private static final CustomAlerts.TimeLatchAlert failedAlert=new CustomAlerts.TimeLatchAlert(Alert.AlertType.WARNING,3.0,"Failed to pickup");
  /**
   * Makes a trajectory using the current position and velocity that goes to the given position with
   * the ending tangent line slope
   *
   * @param drive The drivetrain
   * @param target The target translation, and the direction of movement at the end of the
   *     trajectory
   * @return The trajectory to go to the given pose
   */
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

  /**
   * Makes a trajectory using the current position and velocity that goes to the given position in a
   * straight line
   *
   * @param drive The drivetrain
   * @param target The target translation
   * @return The trajectory to go to the given position
   */
  private static Trajectory makeTrajectory(Drivetrain drive, Translation2d target) {
    return makeTrajectory(
        drive, new Pose2d(target, target.minus(drive.getPosition().getTranslation()).getAngle()));
  }

  public static Command driveToPos(Drivetrain drivetrain, Translation2d target) {
    return new HolonomicTrajectoryFollower(drivetrain, () -> makeTrajectory(drivetrain, target));
  }

  /**
   * Makes a command to shoot into the speaker
   *
   * @param drivetrain The drivetrain
   * @param arm The arm
   * @param shooter The shooter
   * @return A command to rotate to and fire into the speaker
   */
  public static Command speaker(Drivetrain drivetrain, Arm arm, Shooter shooter) {
    RotateTo rt = new RotateTo(drivetrain);
    SpeakerShoot speakerShoot = new SpeakerShoot(drivetrain, arm, rt::setCustomRotation, shooter);
    return speakerShoot.raceWith(rt.repeatedly());
  }

  /**
   * Makes a command to pick up a game piece
   *
   * @param drivetrain The drivetrain
   * @param arm The arm
   * @param shooter The shooter
   * @param pos The game piece index
   * @return A command that drives to the game piece and intakes it, but will also give up after a
   *     second
   */
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
                .andThen(Commands.waitSeconds(1)).andThen(failedAlert::latch));
  }
  /**
   * Makes a command to pick up a game piece. If it is not successful, it will attempt to pick up
   * another game piece
   *
   * @param drivetrain The drivetrain
   * @param arm The arm
   * @param shooter The shooter
   * @param pos The game piece index
   * @param backupPos The game piece index to pick up if the first fails
   * @return A command that drives to the game piece and attempts to intake it. If it fails, it will
   *     attempt to pick up the second game piece
   */
  public static Command pickupWithBackup(
      Drivetrain drivetrain, Arm arm, Shooter shooter, int pos, int backupPos) {
    return pickup(drivetrain, arm, shooter, pos)
        .andThen(
            Commands.either(
                Commands.none(), pickup(drivetrain, arm, shooter, backupPos), shooter::isHolding));
  }
}
