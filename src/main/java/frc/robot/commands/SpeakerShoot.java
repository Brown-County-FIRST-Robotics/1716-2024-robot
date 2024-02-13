package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ShootWhileMove;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Consumer;

public class SpeakerShoot extends Command {
  Drivetrain drive;
  Arm arm;
  Consumer<Optional<Rotation2d>> rotationCommander;
  Shooter shooter;
  boolean firing = false;
  LoggedTunableNumber shooterAngleThreshold = new LoggedTunableNumber("ang threshold", 0.01);
  LoggedTunableNumber botAngleThreshold = new LoggedTunableNumber("bot ang threshold", 0.02);
  LoggedTunableNumber botAngleThresho2ld = new LoggedTunableNumber("shoot fac2222", 0.8);

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
    Pose2d pos = drive.getPosition();
    Translation3d botPose = new Translation3d(pos.getX(),pos.getY(),0.7);//new Pose3d(pos.getX(), pos.getY(), 0,new Rotation3d()).transformBy(new Transform3d(new Translation3d(0.3,0,0.3),new Rotation3d(0,-arm.getAngle().getRadians(),0))).transformBy(new Transform3d(new Translation3d(0.3,0,0.18),new Rotation3d())).getTranslation();
    Logger.recordOutput("PredPose",new Pose3d(botPose,new Rotation3d(0,-arm.getAngle().getRadians(),0)));
    var botAngle=FieldConstants.getSpeaker().minus(botPose).toTranslation2d().getAngle();
    var shooterAngle=new Rotation2d(FieldConstants.getSpeaker().minus(botPose).toTranslation2d().getNorm(),FieldConstants.getSpeaker().minus(botPose).getZ());
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
    shooter.setFiringBlocked(
        botAngleThreshold.get()
                < Math.abs(botAngle.minus(drive.getPosition().getRotation()).getRotations())
            || shooterAngleThreshold.get()
                < Math.abs(shooterAngle.minus(arm.getAngle()).getRotations()));
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
