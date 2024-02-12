package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DualRateLimiter;
import frc.robot.utils.LoggedTunableNumber;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/** A command for manual control */
public class TeleopDrive extends Command {
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;
  private final ProfiledPIDController ppc =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3, 3));
  LoggedTunableNumber p = new LoggedTunableNumber("drP", -50);
  LoggedTunableNumber i = new LoggedTunableNumber("drI", 0);
  LoggedTunableNumber d = new LoggedTunableNumber("drD", 0);
  boolean foc = true;
  boolean locked = false;
  DualRateLimiter xVelLimiter = new DualRateLimiter(4, 100);
  DualRateLimiter yVelLimiter = new DualRateLimiter(4, 100);
  DualRateLimiter omegaLimiter = new DualRateLimiter(6, 100);

  // TEMP CODE
  LoggedDashboardNumber armSetpoint = new LoggedDashboardNumber("Arm Setpoint", 0.0);
  // END TEMP

  public void setCustomRotation(Optional<Rotation2d> customRotation) {
    this.customRotation = customRotation;
  }

  Optional<Rotation2d> customRotation = Optional.empty();

  /**
   * Constructs a new command with a given controller and drivetrain
   *
   * @param drivetrain The drivetrain subsystem
   * @param controller The driver conroller
   */
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
    if (customRotation.isPresent()) {
      ext =
          ppc.calculate(
              customRotation.get().minus(drivetrain.getPosition().getRotation()).getRotations(), 0);
    }
    // TEMP CODE
    //    if (controller.getHID().getRightTriggerAxis() > 0.2) {
    //      if (deadband(controller.getRightX())) {
    //        ext =
    //            ppc.calculate(
    //                drivetrain
    //                    .getPosition()
    //                    .getRotation()
    //                    .minus(
    //                        drivetrain
    //                            .getPosition()
    //                            .getTranslation()
    //                            .minus(FieldConstants.getSpeaker().toTranslation2d())
    //                            .getAngle())
    //                    .getRotations(),
    //                0);
    //      }
    //      arm.setAngle(
    //          new Rotation2d(
    //              drivetrain
    //                  .getPosition()
    //                  .getTranslation()
    //                  .minus(FieldConstants.getSpeaker().toTranslation2d())
    //                  .getNorm(),
    //              FieldConstants.getSpeaker().getZ()));
    //    } else {
    //      arm.setAngle(Rotation2d.fromRotations(0.7));
    //    }
    // END TEMP

    controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, Math.abs(ext / 3.0));
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
              controller.getLeftY() * Constants.Driver.MAX_X_SPEED * slow,
              controller.getLeftX() * Constants.Driver.MAX_Y_SPEED * slow,
              controller.getRightX() * Constants.Driver.MAX_THETA_SPEED * slow + ext);

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
    drivetrain.humanDrive(new ChassisSpeeds());
  }
}
