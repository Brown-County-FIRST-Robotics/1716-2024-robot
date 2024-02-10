package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;
import frc.robot.utils.ShootWhileMove;
import frc.robot.utils.ShootWhileMove.ShootingCommand;

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
  LoggedTunableNumber armAngleOverrideIncrementScale = new LoggedTunableNumber("arm angle override increment scale", 1.0);
  XboxController controller;

  public SpeakerShoot(
      Drivetrain drive,
      Arm arm,
      Consumer<Optional<Rotation2d>> rotationCommander,
      Shooter shooter,
      XboxController overrideController) {
    this.drive = drive;
    this.arm = arm;
    this.rotationCommander = rotationCommander;
    this.shooter = shooter;
    controller = overrideController;
    addRequirements(arm, shooter); // DO NOT add drive
  }
  
  @Override
  public void initialize() {
    shooter.setFiringBlocked(true);
  }

  @Override
  public void execute() {
    ShootingCommand cmd = new ShootingCommand(new Rotation2d(), new Rotation2d(), 10);
    if (!Overrides.disableAutoAiming.get()) {
      Pose2d pos = drive.getPosition();
      Translation3d botPose = new Translation3d(pos.getX(), pos.getY(), 0);
      cmd = ShootWhileMove.calcSimpleCommand(
        botPose.minus(FieldConstants.getSpeaker()),
        ShootWhileMove.getFieldRelativeSpeeds(
            drive.getVelocity(), drive.getPosition().getRotation()));

      if (!Overrides.disableArmAnglePresets.get()) {
        arm.setAngle(cmd.shooterAngle);
      }
      else {
        arm.commandIncrement(Rotation2d.fromRotations(controller.getLeftY() * armAngleOverrideIncrementScale.get()));
      }
      rotationCommander.accept(Optional.of(cmd.botAngle));
      shooter.setFiringBlocked(
          botAngleThreshold.get()
                  < Math.abs(cmd.botAngle.minus(drive.getPosition().getRotation()).getRotations())
              || (shooterAngleThreshold.get()
                  < Math.abs(cmd.shooterAngle.minus(arm.getAngle()).getRotations())
                  || Overrides.disableArmAnglePresets.get()));
    }
    else {
      rotationCommander.accept(Optional.empty());
      arm.commandIncrement(Rotation2d.fromRotations(controller.getLeftY() * armAngleOverrideIncrementScale.get()));
    }
    shooter.commandSpeed(cmd.shooterSpeedMPS);
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
