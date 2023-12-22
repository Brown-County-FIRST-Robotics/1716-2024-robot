package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class TeleopDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final CommandXboxController controller;
  private final Rotation2d lockBand = Rotation2d.fromDegrees(20);
  private final Rotation2d lockRot = Rotation2d.fromDegrees(180);

  private final Rotation2d minRot = lockRot.rotateBy(lockBand.times(-0.5));
  private final Rotation2d maxRot = lockRot.rotateBy(lockBand.times(0.5));
  private final ProfiledPIDController ppc =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3, 3));
  LoggedTunableNumber p = new LoggedTunableNumber("drP", -50);
  LoggedTunableNumber i = new LoggedTunableNumber("drI", 0);
  LoggedTunableNumber d = new LoggedTunableNumber("drD", 0);
  boolean foc = true;
  boolean locked = false;

  static double maxv = 5;
  static double maxa = 10;
  static double overpowered = 1.1;
  static double veldeadband = .1;
  Translation2d vel = new Translation2d();

  public TeleopDrive(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(this.drivetrain);
    ppc.setP(p.get());
    ppc.setI(i.get());
    ppc.setD(d.get());
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()} returns true.)
   */
  static Translation2d dedeadband(Translation2d sp) {
    double len = sp.getNorm();
    if (len < 0.1) {
      return new Translation2d();
    }
    double new_len = (len - 0.1) / (1.0 - 0.1);
    return new Translation2d(new_len, sp.getAngle());
  }

  static Translation2d desaturate(Translation2d inp, double maxVel) {
    return new Translation2d(Math.min(maxVel, inp.getNorm()), inp.getAngle());
  }

  static double powerWeight(double x, double exponent) {
    return x * Math.abs(Math.pow(x, exponent - 1.0));
  }

  static Translation2d powerWeight(Translation2d x, double exponent) {
    return new Translation2d(powerWeight(x.getNorm(), exponent), x.getAngle());
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
    Logger.getInstance().recordOutput("TeleopDrive/ext", ext);
    Translation2d sticks = new Translation2d(controller.getLeftY(), controller.getLeftX());
    Translation2d spsd =
        powerWeight(dedeadband(sticks), 2);
    Translation2d cmdAccel = powerWeight(spsd.minus(vel.div(maxv * overpowered)), 2);

    Translation2d applV = desaturate(vel.plus(desaturate(cmdAccel, maxa).times(0.02)), maxv);
    vel=sticks;//applV;
System.out.printf("stickvel %f %f %s\n", controller.getLeftX(), controller.getLeftX(), vel);
    if (Math.abs(controller.getLeftY()) < 0.1
        && Math.abs(controller.getLeftX()) < 0.1
        && Math.abs(controller.getRightX()) < 0.1) {
      vel = new Translation2d();
      drivetrain.humanDrive(new ChassisSpeeds(0, 0, ext), false);
    } else {
      locked = false;
      drivetrain.humanDrive(
          new ChassisSpeeds(
              applV.getX(),
              applV.getY(),
              controller.getRightX()
                      * Math.abs(controller.getRightX())
                      * Constants.Driver.MAX_THETA_SPEED
                  + ext),
          foc);
    }
    if (controller.getHID().getBackButtonPressed()) {
      drivetrain.setPosition(
          new Pose2d(drivetrain.getPosition().getTranslation(), Rotation2d.fromRotations(0.5)));
    }
    locked = controller.getHID().getXButtonPressed() || locked;
    foc = controller.getHID().getStartButtonPressed() != foc;
    if (p.hasChanged()) {
      ppc.setP(p.get());
    }
    if (i.hasChanged()) {
      ppc.setI(i.get());
    }
    if (d.hasChanged()) {
      ppc.setD(d.get());
    }
    if (locked) {
      drivetrain.lockWheels();
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
