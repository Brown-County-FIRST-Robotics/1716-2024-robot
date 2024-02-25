package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DualRateLimiter;
import frc.robot.utils.HolonomicTrajectoryFollower;
import frc.robot.utils.Overrides;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** A command for manual control */
public class TeleopDrive extends Command {
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;
  boolean foc = true;
  boolean locked = false;
  DualRateLimiter tVelLimiter = new DualRateLimiter(4, 100);
  DualRateLimiter omegaLimiter = new DualRateLimiter(6, 100);

  public void setCustomRotation(Optional<Rotation2d> customRotation) {
    this.customRotation = customRotation;
  }

  Optional<Rotation2d> customRotation = Optional.empty();

  /**
   * Constructs a new command with a given controller and drivetrain
   *
   * @param drivetrain The drivetrain subsystem
   * @param controller The driver controller
   */
  public TeleopDrive(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(this.drivetrain);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /**
   * Checks if the value is in the deadband
   *
   * @param val The value to check
   * @return If it is in the deadband
   */
  static boolean deadband(double val) {
    return Math.abs(val) < 0.1;
  }

  /**
   * Applies a deadband, then scales the resultant value to make output continuous
   *
   * @param val The value
   * @return The value with the deadband applied
   */
  static double deadscale(double val) {
    return deadband(val) ? 0 : (val > 0 ? (val - 0.1) / 0.9 : (val + 0.1) / 0.9);
  }

  @Override
  public void execute() {
    double ext =
        customRotation
            .map(
                rotation2d ->
                    HolonomicTrajectoryFollower.getExt(
                        rotation2d,
                        drivetrain.getPosition().getRotation(),
                        drivetrain.getVelocity().omegaRadiansPerSecond))
            .orElse(0.0);

    Logger.recordOutput("TeleopDrive/ext", ext);
    double slow =
        controller.getHID().getLeftBumper() || controller.getHID().getRightBumper() ? 0.2 : 1.0;

    if (deadband(controller.getLeftY())
        && deadband(controller.getLeftX())
        && deadband(controller.getRightX())) {
      drivetrain.humanDrive(new ChassisSpeeds(0, 0, ext));
    } else {
      locked = false;
      ChassisSpeeds cmd =
          new ChassisSpeeds(
              deadscale(controller.getLeftY()) * Constants.Driver.MAX_X_SPEED * slow,
              deadscale(controller.getLeftX()) * Constants.Driver.MAX_Y_SPEED * slow,
              omegaLimiter.calculate(deadscale(controller.getRightX()) * Constants.Driver.MAX_THETA_SPEED * slow) + ext);
      var cmdAsTranslation=new Translation2d(cmd.vxMetersPerSecond,cmd.vyMetersPerSecond);
      var realNorm=tVelLimiter.calculate(cmdAsTranslation.getNorm());
      var realCmdAsTranslation=new Translation2d(realNorm,cmdAsTranslation.getAngle());
      ChassisSpeeds sp =
          new ChassisSpeeds(
              -cmd.vxMetersPerSecond, -cmd.vyMetersPerSecond, -cmd.omegaRadiansPerSecond);
      if (foc) {
        Rotation2d rot =
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
                    == DriverStation.Alliance.Red
                ? drivetrain.getPosition().getRotation()
                : drivetrain.getPosition().getRotation().rotateBy(Rotation2d.fromRotations(0.5));
        sp =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                    cmd.vxMetersPerSecond, cmd.vyMetersPerSecond, -cmd.omegaRadiansPerSecond),
                rot);
      }

      drivetrain.humanDrive(sp);
    }
    if (controller.getHID().getBackButtonPressed()) {
      drivetrain.setPosition(
          new Pose2d(drivetrain.getPosition().getTranslation(), Rotation2d.fromRotations(0.5)));
    }
    locked = controller.getHID().getXButtonPressed() || locked;
    foc = Overrides.useFieldOriented.get();
    if (locked) {
      drivetrain.lockWheels();
    }

    Logger.recordOutput("TeleopDrive/locked", locked);
    Logger.recordOutput("TeleopDrive/foc", foc);
  }

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an
   * operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally --
   * that it is called when {@link #isFinished()} returns true -- or when it is
   * interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a
   * motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.humanDrive(new ChassisSpeeds());
  }
}
