package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DualRateLimiter;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class TeleopDrive extends Command {
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;
  private final Rotation2d lockBand = Rotation2d.fromDegrees(20);
  private final Rotation2d lockRot = Rotation2d.fromDegrees(180);

  private final Rotation2d minRot = lockRot.rotateBy(lockBand.times(-0.5));
  private final Rotation2d maxRot = lockRot.rotateBy(lockBand.times(0.5));
  private final ProfiledPIDController ppc =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3, 3));
  LoggedTunableNumber p = new LoggedTunableNumber("drP", -100);
  LoggedTunableNumber i = new LoggedTunableNumber("drI", 0);
  LoggedTunableNumber d = new LoggedTunableNumber("drD", 0);
  boolean foc = true;
  boolean locked = false;
  DualRateLimiter xVelLimiter = new DualRateLimiter(4, 100);
  DualRateLimiter yVelLimiter = new DualRateLimiter(4, 100);
  DualRateLimiter omegaLimiter = new DualRateLimiter(6, 100);

  public TeleopDrive(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(this.drivetrain);
    p.attach(ppc::setP);
    i.attach(ppc::setI);
    d.attach(ppc::setD);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {}

  static boolean deadband(double x) {
    return Math.abs(x) < 0.1;
  }

  @Override
  public void execute() {
    double ext = 0;
    if ((drivetrain.getPosition().getRotation().minus(minRot).getRotations() + 1.0) % 1.0
            < lockBand.getRotations()
        && !controller.getHID().getRightStickButton()) {
      ext += ppc.calculate(drivetrain.getPosition().getRotation().minus(lockRot).getRotations(), 0);
    }
    controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, Math.abs(ext / 3.0));
    Logger.recordOutput("TeleopDrive/ext", ext);

    if (deadband(controller.getLeftY())
        && deadband(controller.getLeftX())
        && deadband(controller.getRightX())) {
      drivetrain.humanDrive(new ChassisSpeeds(0, 0, ext), false);
    } else {
      locked = false;
      drivetrain.humanDrive(
          new ChassisSpeeds(
              xVelLimiter.calculate(
                  controller.getLeftY()
                      * Math.abs(Math.pow(controller.getLeftY(), 2))
                      * Constants.Driver.MAX_X_SPEED),
              yVelLimiter.calculate(
                  controller.getLeftX()
                      * Math.abs(Math.pow(controller.getLeftX(), 2))
                      * Constants.Driver.MAX_Y_SPEED),
              omegaLimiter.calculate(
                      controller.getRightX()
                          * Math.abs(controller.getRightX())
                          * Constants.Driver.MAX_THETA_SPEED)
                  + ext),
          foc);
    }
    if (controller.getHID().getBackButtonPressed()) {
      drivetrain.setPosition(
          new Pose2d(drivetrain.getPosition().getTranslation(), Rotation2d.fromRotations(0.5)));
    }
    locked = controller.getHID().getXButtonPressed() || locked;
    foc = controller.getHID().getStartButtonPressed() != foc;
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
    drivetrain.humanDrive(new ChassisSpeeds(), false);
  }
}
