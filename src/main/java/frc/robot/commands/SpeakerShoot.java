package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ShootWhileMove;
import java.util.Optional;
import java.util.function.Consumer;

public class SpeakerShoot extends Command {
  Drivetrain drive;
  Arm arm;
  Consumer<Optional<Rotation2d>> rotationCommander;
  Shooter shooter;
  boolean firing = false;
  LoggedTunableNumber shooterAngleThreshold = new LoggedTunableNumber("ang threshold", 0.05);
  LoggedTunableNumber botAngleThreshold = new LoggedTunableNumber("bot ang threshold", 0.05);

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
  }

  @Override
  public void execute() {
    Pose2d pos=drive.getPosition().plus(new Transform2d(-1.3,0, new Rotation2d()));
    Translation3d botPose =
        new Translation3d(pos.getX(), pos.getY(), 0);
    var cmd =
        ShootWhileMove.calcSimpleCommand(
            botPose.minus(FieldConstants.getSpeaker()),
            ShootWhileMove.getFieldRelativeSpeeds(
                drive.getVelocity(), drive.getPosition().getRotation()));
    shooter.commandSpeed(cmd.shooterSpeedMPS);
    rotationCommander.accept(Optional.of(cmd.botAngle));
    arm.setAngle(cmd.shooterAngle);
    shooter.setFiringBlocked(
        botAngleThreshold.get()
                < Math.abs(cmd.botAngle.minus(drive.getPosition().getRotation()).getRotations())
            || shooterAngleThreshold.get()
                < Math.abs(cmd.shooterAngle.minus(arm.getAngle()).getRotations()));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    arm.commandNeutral();
    rotationCommander.accept(Optional.empty());
  }

  @Override
  public boolean isFinished() {
    return firing && !shooter.isHolding();
  }
}
