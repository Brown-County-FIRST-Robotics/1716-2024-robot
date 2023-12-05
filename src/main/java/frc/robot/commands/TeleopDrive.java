package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class TeleopDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;
  boolean foc = true;
  boolean locked = false;

  public TeleopDrive(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(this.drivetrain);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()} returns true.)
   */
  static boolean deadband(double x) {
    return Math.abs(x) < 0.1;
  }

  @Override
  public void execute() {
    if (deadband(controller.getLeftY())
        && deadband(controller.getLeftX())
        && deadband(controller.getRightX())
        && !locked) {
      drivetrain.humanDrive(new ChassisSpeeds(), false);

    } else {
      locked = false;
      drivetrain.humanDrive(new ChassisSpeeds(
          controller.getLeftY()
              * Math.abs(Math.pow(controller.getLeftY(), 2))
              * Constants.Driver.MAX_X_SPEED,
          controller.getLeftX()
              * Math.abs(Math.pow(controller.getLeftX(), 2))
              * Constants.Driver.MAX_Y_SPEED,
          controller.getRightX()
              * Math.abs(controller.getRightX())
              * Constants.Driver.MAX_THETA_SPEED),
          foc);
      foc = controller.getHID().getStartButtonPressed() != foc;
    }
    if (controller.getHID().getBackButtonPressed()) {
      drivetrain.setPosition(
          new Pose2d(drivetrain.getPosition().getTranslation(), Rotation2d.fromRotations(0.5)));
    }
    if (controller.getHID().getXButtonPressed()) {
      locked = true;
    }


    Logger.getInstance().recordOutput("TeleopDrive/locked", locked);
    Logger.getInstance().recordOutput("TeleopDrive/foc", foc);
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
    drivetrain.humanDrive(new ChassisSpeeds(), false);
  }
}
