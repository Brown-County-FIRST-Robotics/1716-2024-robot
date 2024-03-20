package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DualRateLimiter;
import frc.robot.utils.HolonomicTrajectoryFollower;
import frc.robot.utils.Overrides;
import frc.robot.utils.Vector;

import java.lang.System;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** A command for manual control */
public class TeleopDrive extends Command {
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;

  boolean doFieldOriented = true;
  boolean locked = false; //point wheels towards center in x pattern so we can't be pushed
  DualRateLimiter translationLimiter = new DualRateLimiter(6, 100); //translational velocity limiter
  DualRateLimiter rotationLimiter = new DualRateLimiter(9, 100); //angular velocity limiter (omega)

  Optional<Rotation2d> customRotation = Optional.empty(); //used for auto align; if empty, no target is set

  private static double deadbandSize = 0.08;

  double slowModeSpeedModifier = 0.0;
  double customAngleModifier = 0.0;
  ChassisSpeeds commandedSpeeds = new ChassisSpeeds(0, 0, 0);
  ChassisSpeeds finalSpeeds = new ChassisSpeeds(0, 0, 0);

  /**
   * Constructs a new command with a given controller and drivetrain
   *
   * @param drivetrain The drivetrain subsystem
   * @param controller The driver controller, used for various inputs
   */
  public TeleopDrive(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(this.drivetrain);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    translationLimiter.reset(0);
    rotationLimiter.reset(0);
    customRotation = Optional.empty();
  }
  
  @Override
  public void execute() {
    customAngleModifier =
        customRotation
            .map(
                rotation2d ->
                    HolonomicTrajectoryFollower.getExt(
                        rotation2d,
                        drivetrain.getPosition().getRotation(),
                        drivetrain.getVelocity().omegaRadiansPerSecond))
            .orElse(0.0); //The velocity added to the rotation to apply the custom angle

    Logger.recordOutput("TeleopDrive/ext", customAngleModifier);
    slowModeSpeedModifier =
        controller.getHID().getLeftBumper() || controller.getHID().getRightBumper() ? 0.2 : 1.0;

    // if (withinDeadband(controller.getLeftY())
    //     && withinDeadband(controller.getLeftX())
    //     && withinDeadband(controller.getRightX())) {
    //   drivetrain.humanDrive(new ChassisSpeeds(0, 0, customAngleModifier));
    // } else {
      locked = false;
      commandedSpeeds =
          new ChassisSpeeds(
              deadscale(controller.getLeftY())
                  * slowModeSpeedModifier,
              deadscale(controller.getLeftX())
                  * slowModeSpeedModifier,
              rotationLimiter.calculate(
                      deadscale(controller.getRightX()) * Constants.Driver.MAX_THETA_SPEED * slowModeSpeedModifier)
                  + customAngleModifier); // This needs to be a different type, the speeds need to be percentage at this step, not velocity

      if (doFieldOriented) {
        Rotation2d currentRotation =
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
                    == DriverStation.Alliance.Red
                ? drivetrain.getPosition().getRotation()
                : drivetrain.getPosition().getRotation().rotateBy(Rotation2d.fromRotations(0.5)); //current rotation relative to the driver
        commandedSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                    commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond, -commandedSpeeds.omegaRadiansPerSecond),
                currentRotation);
      }
      else {
        commandedSpeeds =
          new ChassisSpeeds(
              -commandedSpeeds.vxMetersPerSecond,
              -commandedSpeeds.vyMetersPerSecond,
              -commandedSpeeds.omegaRadiansPerSecond);
      }

      Vector commandedVector = new Vector(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
      commandedVector.setNorm(clamp(commandedVector.getNorm(), 1.0));
      commandedVector.setNorm(commandedVector.getNorm() * Math.abs(commandedVector.getNorm())); //square it
      commandedVector.setNorm(commandedVector.getNorm() * Constants.Driver.MAX_SPEED); //convert to m/s from percent
      
      Vector currentVector = new Vector(drivetrain.getVelocity().vxMetersPerSecond, drivetrain.getVelocity().vyMetersPerSecond);
      Vector velocityChange = commandedVector.minus(currentVector);
      double frictionClampedVelocityChange = clamp(velocityChange.getNorm(), Constants.Driver.MAX_FRICTION_ACCELERATION / 50); //TODO: CHANGE NAME
      // Logger.recordOutput("before angle", commandedVector.getAngle());
      // Logger.recordOutput("before norm", commandedVector.getNorm());
      Vector cappedAcceleration = new Vector(frictionClampedVelocityChange, velocityChange.getAngle());
      commandedVector = currentVector.plus(cappedAcceleration);
      // Logger.recordOutput("after angle", commandedVector.getAngle());
      // Logger.recordOutput("after norm", commandedVector.getNorm());

      drivetrain.humanDrive(new ChassisSpeeds(commandedVector.getX(), commandedVector.getY(), commandedSpeeds.omegaRadiansPerSecond));
    // }

    if (controller.getHID().getBackButtonPressed()) {
      drivetrain.setPosition(
          new Pose2d(drivetrain.getPosition().getTranslation(), Rotation2d.fromRotations(0.5)));
    }

    doFieldOriented = Overrides.useFieldOriented.get();
    locked = controller.getHID().getXButtonPressed() || locked;
    if (locked) {
      drivetrain.lockWheels();
    }

    Logger.recordOutput("TeleopDrive/locked", locked);
    Logger.recordOutput("TeleopDrive/foc", doFieldOriented);
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

  public void setCustomRotation(Optional<Rotation2d> customRotation) {
    this.customRotation = customRotation;
  }

  /**
   * Checks if the value is in the deadband
   *
   * @param val The value to check
   * @return Whether val is in the deadband
   */
  static boolean withinDeadband(double val) {
    return Math.abs(val) < deadbandSize;
  }

  /**
   * Applies a deadband, then scales the resultant value to make output continuous
   *
   * @param val The value to scale
   * @return The value with the deadband applied
   */
  static double deadscale(double val) {
    return withinDeadband(val) ? 0 : (val > 0 ? (val - deadbandSize) / (1 - deadbandSize) : (val + deadbandSize) / (1 - deadbandSize));
  }

  // Clamps the value to the max, applies in both negative and positive
  private double clamp(double x, double max) {
    if (x > max) {
      x = max;
    }
    else if (x < -max) {
      x = -max;
    }
    return x;
  }
}
