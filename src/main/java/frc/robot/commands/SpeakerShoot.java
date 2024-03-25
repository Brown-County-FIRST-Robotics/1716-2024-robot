package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;
import frc.robot.utils.ShootWhileMove;
import java.util.Optional;
import java.util.function.Consumer;

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
              .transformBy(new Transform3d(new Translation3d(0.3, 0, 0.115), new Rotation3d()))
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
    var cmd =
        ShootWhileMove.calcCommandWithKinematics(
            pos.getTranslation(),
            FieldConstants.getSpeaker(),
            ShootWhileMove.getFieldRelativeSpeeds(
                drive.getVelocity(), drive.getPosition().getRotation()),
            kinematics);
    shooter.setSpeed(9.88); // Max speed
    rotationCommander.accept(Optional.of(cmd.botAngle));
    arm.setAngle(cmd.shooterAngle);
    if (!Double.isNaN(cmd.shooterAngle.getRadians())) {
      arm.setAngle(cmd.shooterAngle);
    }
    // Prevent firing if angles are not close enough
    boolean blocked =
        0.006 < Math.abs(cmd.botAngle.minus(drive.getPosition().getRotation()).getRotations())
            || shooterAngleThreshold.get()
                < Math.abs(cmd.shooterAngle.minus(arm.getAngle()).getRotations())
            || drive.getVelocity().omegaRadiansPerSecond > 0.2;

    if (blocked) {
      ft.restart();
    }
    boolean rb = !ft.hasElapsed(0.5);
    shooter.setFiringBlocked(rb);
    firing = firing || (!rb);
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
