package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
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
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class SpeakerShoot extends Command {
  Drivetrain drive;
  Arm arm;
  Consumer<Optional<Rotation2d>> rotationCommander;
  Shooter shooter;
  boolean firing = false;
  LoggedTunableNumber shooterAngleThreshold = new LoggedTunableNumber("ang threshold", 0.01);
  LoggedTunableNumber botAngleThreshold = new LoggedTunableNumber("bot ang threshold", 0.02);
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
    Pose2d pos = drive.getPosition();
    Translation3d botPose =
        new Pose3d(pos.getX(), pos.getY(), 0, new Rotation3d(0, 0, pos.getRotation().getRadians()))
            .transformBy(
                new Transform3d(
                    new Translation3d(0.3, 0, 0.3),
                    new Rotation3d(0, -arm.getAngle().getRadians(), 0)))
            .transformBy(new Transform3d(new Translation3d(0.3, 0, 0.18), new Rotation3d()))
            .getTranslation();

    var shooterAngle =
        new Rotation2d(
            FieldConstants.getSpeaker().minus(botPose).toTranslation2d().getNorm(),
            FieldConstants.getSpeaker().minus(botPose).getZ());

    // Iterate to find the best shooter angle. Untested
    //    for (int i = 0; i < 10; i++) {
    //      botPose = new Pose3d(pos.getX(), pos.getY(), 0,new
    // Rotation3d(0,0,pos.getRotation().getRadians()))
    //            .transformBy(new Transform3d(new Translation3d(0.3,0,0.3),new
    // Rotation3d(0,-shooterAngle.getRadians(),0)))
    //            .transformBy(new Transform3d(new Translation3d(0.3,0,0.18),new
    // Rotation3d())).getTranslation();
    //      shooterAngle=new
    // Rotation2d(FieldConstants.getSpeaker().minus(botPose).toTranslation2d().getNorm(),FieldConstants.getSpeaker().minus(botPose).getZ());
    //    }

    var botAngle = FieldConstants.getSpeaker().minus(botPose).toTranslation2d().getAngle();
    Logger.recordOutput(
        "PredPose", new Pose3d(botPose, new Rotation3d(0, -shooterAngle.getRadians(), 0)));
    shooter.commandSpeed(9.88);
    rotationCommander.accept(Optional.of(botAngle));
    arm.setAngle(shooterAngle);
    //    Bad Shooting While Moving Code
    //    var cmd =
    //        ShootWhileMove.calcSimpleCommand(
    //            botPose.minus(FieldConstants.getSpeaker()),
    //            ShootWhileMove.getFieldRelativeSpeeds(
    //                drive.getVelocity(), drive.getPosition().getRotation()));
    //    shooter.commandSpeed(cmd.shooterSpeedMPS);
    //    Logger.recordOutput("Drive/CMDState",new Pose2d(pos.getTranslation(),cmd.botAngle));
    //    rotationCommander.accept(Optional.of(cmd.botAngle));
    //    arm.setAngle(cmd.shooterAngle.times(botAngleThresho2ld.get()));
    boolean blocked =
        botAngleThreshold.get()
                < Math.abs(botAngle.minus(drive.getPosition().getRotation()).getRotations())
            || shooterAngleThreshold.get()
                < Math.abs(shooterAngle.minus(arm.getAngle()).getRotations());
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
    return !Overrides.disableAutoAiming.get() && (firing && !shooter.isHolding());
  }
}
