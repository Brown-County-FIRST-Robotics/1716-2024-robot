package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;
import java.util.Optional;
import java.util.function.Consumer;

public class SimpleSpeakerShoot extends Command {
  Drivetrain drive;
  Arm arm;
  Consumer<Optional<Rotation2d>> rotationCommander;
  Shooter shooter;
  boolean firing = false;
  LoggedTunableNumber shooterAngleThreshold = new LoggedTunableNumber("ang threshold", 0.01);
  LoggedTunableNumber botAngleThreshold = new LoggedTunableNumber("bot ang threshold", 0.02);
  XboxController overrideController;

  public SimpleSpeakerShoot(
      Drivetrain drive,
      Arm arm,
      Consumer<Optional<Rotation2d>> rotationCommander,
      Shooter shooter,
      XboxController overrideController) {
    this.drive = drive;
    this.arm = arm;
    this.rotationCommander = rotationCommander;
    this.shooter = shooter;
    this.overrideController = overrideController;
    addRequirements(arm, shooter); // DO NOT add drive
  }

  @Override
  public void initialize() {
    shooter.setFiringBlocked(true);
  }

  @Override
  public void execute() {
    // Calculate position of the tip of the shooter
    Pose2d pos = drive.getPosition();
    Translation3d botPose =
        new Pose3d(pos.getX(), pos.getY(), 0, new Rotation3d(0, 0, pos.getRotation().getRadians()))
            .transformBy(
                new Transform3d(
                    new Translation3d(0.3, 0, 0.3),
                    new Rotation3d(0, -arm.getAngle().getRadians(), 0)))
            .transformBy(new Transform3d(new Translation3d(0.3, 0, 0.18), new Rotation3d()))
            .getTranslation();
    var botAngle = FieldConstants.getSpeaker().minus(botPose).toTranslation2d().getAngle();
    shooter.setSpeed(9.88);
    boolean blocked = false;

    // Auto alignment code
    if (Overrides.disableAutoAlign.get()) {
      rotationCommander.accept(Optional.empty());
      // Don't block firing if auto alignment is disabled
    } else {
      rotationCommander.accept(Optional.of(botAngle));
      blocked =
          botAngleThreshold.get()
              < Math.abs(botAngle.minus(drive.getPosition().getRotation()).getRotations());
    }
    if (Overrides.disableArmAnglePresets.get()) {
      arm.commandIncrement(
          Rotation2d.fromRotations(
              overrideController.getLeftY() * Overrides.armAngleOverrideIncrementScale.get()));
      blocked = blocked || (!overrideController.getAButton()); // Use the A button to fire
    } else {
      Rotation2d shooterAngle = Rotation2d.fromRadians(1);
      arm.setAngle(shooterAngle);
      blocked =
          blocked
              || shooterAngleThreshold.get()
                  < Math.abs(shooterAngle.minus(arm.getAngle()).getRotations());
    }
    shooter.setFiringBlocked(blocked);
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
