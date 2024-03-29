package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;
import frc.robot.utils.ShootWhileMove;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class SpeakerShoot extends Command {
  Drivetrain drive;
  Arm arm;
  Consumer<Optional<Rotation2d>> rotationCommander;
  Shooter shooter;
  boolean firing = false;
  LoggedTunableNumber shooterAngleThreshold = new LoggedTunableNumber("ang threshold", 0.003);
  LoggedTunableNumber botAngleThreshold = new LoggedTunableNumber("bot ang threshold", 0.008);
  LoggedTunableNumber sp = new LoggedTunableNumber("Shooter Speed", 11.3);
  Timer ft = new Timer();
  private static final ShootWhileMove.ShooterKinematics kinematics =
      (cmd, botPose) ->
          new Pose3d(
                  botPose.getX(),
                  botPose.getY(),
                  0,
                  new Rotation3d(0, 0, cmd.botAngle.getRadians()))
              .transformBy(
                  new Transform3d(
                      new Translation3d(11 * 0.0254, 0, 10 * 0.0254),
                      new Rotation3d(0, -cmd.shooterAngle.getRadians(), 0)))
              .transformBy(new Transform3d(new Translation3d(0.33, 0, 0.155), new Rotation3d()))
              .getTranslation();

  public SpeakerShoot(
      Drivetrain drive,
      Arm arm,
      Consumer<Optional<Rotation2d>> rotationCommander,
      Shooter shooter) {
    this.drive = drive;
    this.arm = arm;
    this.rotationCommander = rotationCommander;
    this.shooter = shooter;
    addRequirements(arm, shooter); // DO NOT add drive
  }

  @Override
  public void initialize() {
    shooter.setFiringBlocked(true);
    shooter.shoot(-4000, 4000);
    ft.restart();
  }

  @Override
  public void execute() {
    // Calculates position of the tip of the shooter
    Pose2d pos = drive.getPosition();
    Rotation2d angleToSpeaker =
        FieldConstants.getSpeaker()
            .toTranslation2d()
            .minus(pos.getTranslation())
            .getAngle()
            .minus(Rotation2d.fromDegrees(180))
            .unaryMinus();
    var cmd =
        ShootWhileMove.calcCommandWithKinematics(
            pos.getTranslation(),
            FieldConstants.getSpeaker()
                .plus(new Translation3d(0, angleToSpeaker.getDegrees() / 200, 0)),
            ShootWhileMove.getFieldRelativeSpeeds(
                drive.getVelocity(), drive.getPosition().getRotation()),
            kinematics);
    shooter.setSpeed(9.88); // Max speed
    Logger.recordOutput(
        "SHOOTINGTO",
        new Pose3d(
            FieldConstants.getSpeaker()
                .plus(new Translation3d(0, angleToSpeaker.getDegrees() / 200, 0)),
            new Rotation3d()));
    rotationCommander.accept(Optional.of(cmd.botAngle));
    if (Double.isNaN(cmd.shooterAngle.getRadians())) {
      XboxController driver = new XboxController(0);
      Commands.runOnce(() -> driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5))
          .andThen(Commands.waitSeconds(0.04))
          .andThen(() -> driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0))
          .repeatedly()
          .withTimeout(0.5)
          .schedule();
      cancel();
    } else {
      cmd.shooterAngle = cmd.shooterAngle.minus(Rotation2d.fromDegrees(6));
      arm.setAngle(cmd.shooterAngle);
    }
    // Prevent firing if angles are not close enough
    boolean blocked =
        0.01 < Math.abs(cmd.botAngle.minus(drive.getPosition().getRotation()).getRotations())
            || shooterAngleThreshold.get()
                < Math.abs(cmd.shooterAngle.minus(arm.getAngle()).getRotations())
            || drive.getVelocity().omegaRadiansPerSecond > 0.5;

    shooter.setFiringBlocked(blocked);
    firing = firing || (!blocked);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    arm.commandNeutral();
    rotationCommander.accept(Optional.empty());
  }

  @Override
  public boolean isFinished() {
    return !Overrides.disableAutoAiming.get() && (firing && !shooter.isHolding());
  }
}
