package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.HolonomicTrajectoryFollower;
import frc.robot.utils.ShootWhileMove;
import java.util.List;
import java.util.Optional;

/** A bunch of static factory methods for creating auto routines */
public class AutoBuilder {
  private static final CustomAlerts.TimeLatchAlert failedAlert =
      new CustomAlerts.TimeLatchAlert(Alert.AlertType.WARNING, 3.0, "Failed to pickup");
  private final Arm arm;
  private final Drivetrain drivetrain;
  private final Shooter shooter;

  public AutoBuilder(Drivetrain drivetrain, Arm arm, Shooter shooter) {
    this.arm = arm;
    this.drivetrain = drivetrain;
    this.shooter = shooter;
  }

  /**
   * Makes a trajectory using the current position and velocity that goes to the given position with
   * the ending tangent line slope
   *
   * @param target The target translation, and the direction of movement at the end of the
   *     trajectory
   * @return The trajectory to go to the given pose
   */
  public Trajectory makeTrajectory(Pose2d target) {
    return makeTrajectory(target, 3, 5);
  }

  /**
   * Makes a trajectory
   *
   * @param target The target pose
   * @param vel The maximum velocity
   * @param accel The maximum acceleration
   * @return The trajectory that goes from the current position to the goal, obeying the given
   *     constraints
   */
  public Trajectory makeTrajectory(Pose2d target, double vel, double accel) {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(vel, accel);
    Rotation2d realAng;
    if (ShootWhileMove.getFieldRelativeSpeeds(
                drivetrain.getVelocity(), drivetrain.getPosition().getRotation())
            .getNorm()
        < 0.01) {
      realAng = target.getTranslation().minus(drivetrain.getPosition().getTranslation()).getAngle();
    } else {
      var speed =
          ShootWhileMove.getFieldRelativeSpeeds(
              drivetrain.getVelocity(), drivetrain.getPosition().getRotation());
      trajectoryConfig.setStartVelocity(speed.getNorm());
      realAng = speed.getAngle();
    }
    return TrajectoryGenerator.generateTrajectory(
        new Pose2d(drivetrain.getPosition().getTranslation(), realAng),
        List.of(),
        target,
        trajectoryConfig);
  }

  /**
   * Makes a trajectory using the current position and velocity that goes to the given position in a
   * straight line
   *
   * @param target The target translation
   * @return The trajectory to go to the given position
   */
  private Trajectory makeTrajectory(Translation2d target) {
    return makeTrajectory(
        new Pose2d(target, target.minus(drivetrain.getPosition().getTranslation()).getAngle()));
  }

  /**
   * Returns a command to drive to a position
   *
   * @param target The target to drive to
   * @return A command that drives to the target
   */
  public Command driveToPos(Translation2d target) {
    return new HolonomicTrajectoryFollower(drivetrain, () -> makeTrajectory(target));
  }
  /**
   * Returns a command to drive to a position
   *
   * @param target The target to drive to
   * @return A command that drives to the target
   */
  public Command driveToPos(Pose2d target) {
    return new HolonomicTrajectoryFollower(drivetrain, () -> makeTrajectory(target));
  }

  /**
   * Makes a command to shoot into the speaker
   *
   * @return A command to rotate to and fire into the speaker
   */
  public Command speaker() {
    RotateTo rt = new RotateTo(drivetrain);
    SpeakerShoot speakerShoot = new SpeakerShoot(drivetrain, arm, rt::setCustomRotation, shooter);
    return speakerShoot.raceWith(rt.repeatedly());
  }

  /**
   * Makes a command to pick up a game piece
   *
   * @param pos The game piece index
   * @return A command that drives to the game piece and intakes it, but will also give up after a
   *     second
   */
  public Command pickup(int pos) {
    Translation2d target = FieldConstants.getGamePiece(pos);
    HolonomicTrajectoryFollower trajectoryCommand =
        new HolonomicTrajectoryFollower(
            drivetrain,
            () ->
                makeTrajectory(
                    new Pose2d(target, FieldConstants.flip(new Rotation2d())),
                    (pos == 0) ? 1 : 2,
                    (pos == 0) ? 1 : 2));
    HolonomicTrajectoryFollower drive2 =
        new HolonomicTrajectoryFollower(
            drivetrain,
            () ->
                makeTrajectory(
                    new Pose2d(
                        target.minus(
                            (new Translation2d(
                                (pos == 0) ? 0.5 : 1, FieldConstants.flip(new Rotation2d())))),
                        FieldConstants.flip(new Rotation2d()))));
    return Intake.fromFloor(shooter, arm)
        .raceWith(
            drive2
                .onlyWhile(
                    () ->
                        drivetrain
                                    .getPosition()
                                    .getTranslation()
                                    .getDistance(
                                        target.minus(
                                            (new Translation2d(
                                                (pos == 0) ? 0.5 : 1,
                                                FieldConstants.flip(new Rotation2d())))))
                                > 0.3
                            || Math.abs(
                                    drivetrain
                                        .getPosition()
                                        .getRotation()
                                        .minus(FieldConstants.flip(new Rotation2d()))
                                        .getDegrees())
                                > 7)
                .alongWith(
                    Commands.runOnce(
                        () ->
                            drive2.setCustomRotation(
                                Optional.of(FieldConstants.flip(new Rotation2d())))))
                .andThen(
                    trajectoryCommand.alongWith(
                        Commands.run(
                            () ->
                                trajectoryCommand.setCustomRotation(
                                    Optional.of(FieldConstants.flip(new Rotation2d()))))))
                .andThen(Commands.waitSeconds(1))
                .andThen(failedAlert::latch));
  }
  /**
   * Makes a command to pick up a game piece. If it is not successful, it will attempt to pick up
   * another game piece
   *
   * @param pos The game piece index
   * @param backupPos The game piece index to pick up if the first fails
   * @return A command that drives to the game piece and attempts to intake it. If it fails, it will
   *     attempt to pick up the second game piece
   */
  public Command pickupWithBackup(int pos, int backupPos) {
    return pickup(pos)
        .andThen(Commands.either(Commands.none(), pickup(backupPos), shooter::isHolding));
  }
}
