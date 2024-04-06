// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMUIO;
import frc.robot.subsystems.IMUIONavx;
import frc.robot.subsystems.IMUIOPigeon;
import frc.robot.subsystems.IMUIOSim;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkFlex;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.mecanum.MecanumDrivetrain;
import frc.robot.subsystems.mecanum.MecanumIO;
import frc.robot.subsystems.mecanum.MecanumIOSpark;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSparkFX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOSecondSight;
import frc.robot.utils.AutoFactories;
import frc.robot.utils.HolonomicTrajectoryFollower;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;
import frc.robot.utils.shuffleboard.LoggedShuffleBoardChooser;
import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController secondController = new CommandXboxController(1);
  private final Drivetrain driveSys;
  private Arm arm;
  private Shooter shooter;
  private Climber climber;
  LoggedShuffleBoardChooser<Command> autoChooser =
      new LoggedShuffleBoardChooser<>("Pre Match", "Auto chooser");
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (WhoAmI.mode != WhoAmI.Mode.REPLAY) {
      switch (WhoAmI.bot) {
        case MECHBASE:
          driveSys = new MecanumDrivetrain(new MecanumIOSpark(1, 2, 3, 4), new IMUIOPigeon(20));
          break;
        case SIMSWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIOSim(0), 0),
                  new Module(new ModuleIOSim(1), 1),
                  new Module(new ModuleIOSim(2), 2),
                  new Module(new ModuleIOSim(3), 3),
                  new IMUIOSim());
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIOSparkFX(22, 10, "FL"), 0),
                  new Module(new ModuleIOSparkFX(24, 12, "FR"), 1),
                  new Module(new ModuleIOSparkFX(21, 13, "BL"), 2),
                  new Module(new ModuleIOSparkFX(20, 11, "BR"), 3),
                  new IMUIONavx());
          var vision =
              new Vision(
                  driveSys,
                  new Transform3d[] {
                    new Transform3d(
                        new Translation3d(8 * 0.0254, 11 * 0.0254, 22 * 0.0254),
                        new Rotation3d(0, -8.0 * Math.PI / 180, 0))
                  },
                  new VisionIO[] {new VisionIOSecondSight("SS_LAPTOP", "0")});
          break;
        default:
          driveSys = new MecanumDrivetrain(new MecanumIOSpark(1, 2, 3, 4), new IMUIONavx());
      }
      for (var appendage : WhoAmI.appendages) {
        switch (appendage) {
          case SIM_ARM:
            arm = new Arm(new ArmIOSim());
            break;
          case ARM:
            arm = new Arm(new ArmIOSparkFlex(9));
            break;
          case SHOOTER:
            shooter = new Shooter(new ShooterIOSparkFlexes(58, 57), new FeederIOSpark550(41, 0, 1));
            break;
          case CLIMBER:
            climber = new Climber(new ClimberIOSparkMaxes(29, 35, 6, 7, 8, 9)); // TODO: UPDATE IDs
        }
      }
    } else {
      switch (WhoAmI.bot) {
        case SIMSWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIO() {}, 0),
                  new Module(new ModuleIO() {}, 1),
                  new Module(new ModuleIO() {}, 2),
                  new Module(new ModuleIO() {}, 3),
                  new IMUIO() {});
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIO() {}, 0),
                  new Module(new ModuleIO() {}, 1),
                  new Module(new ModuleIO() {}, 2),
                  new Module(new ModuleIO() {}, 3),
                  new IMUIO() {});
          var vision =
              new Vision(
                  driveSys,
                  new Transform3d[] {
                    new Transform3d(
                        new Translation3d(0 * 0.0254, 0 * 0.0254, 22 * 0.0254),
                        new Rotation3d(0, -12 * Math.PI / 180, 0))
                  },
                  new VisionIO[] {new VisionIO() {}});
          break;
        default:
          driveSys = new MecanumDrivetrain(new MecanumIO() {}, new IMUIO() {});
      }
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {}, new FeederIO() {});
    }
    configureBindings();
  }

  public void configureAutos() {
    autoChooser.addDefaultOption("None", Commands.none());

    autoChooser.addOption(
        "Leave zone",
        Commands.defer(
            () ->
                AutoFactories.driveToPos(
                    driveSys,
                    new Pose2d(
                            driveSys.getPosition().getTranslation(),
                            FieldConstants.flip(new Rotation2d()))
                        .plus(new Transform2d(3, 0, Rotation2d.fromDegrees(0)))
                        .getTranslation()),
            Set.of(driveSys)));
    var returningShotPos = FieldConstants.flip(new Translation2d(1.5, 5.5));
    var shootingFromPosition = FieldConstants.flip(new Translation2d(2.2, 5.5));
    autoChooser.addOption(
        "Drive Shoot Pickup 0 drive shoot",
        AutoFactories.driveToPos(driveSys, shootingFromPosition)
            .onlyIf(
                () -> driveSys.getPosition().getTranslation().getDistance(returningShotPos) > 0.5)
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 0))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));
    autoChooser.addOption(
        "Drive Shoot Pickup 1 drive shoot",
        AutoFactories.driveToPos(driveSys, shootingFromPosition)
            .onlyIf(
                () -> driveSys.getPosition().getTranslation().getDistance(returningShotPos) > 0.5)
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 1))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));

    autoChooser.addOption(
        "Drive Shoot Pickup 2 drive shoot",
        AutoFactories.driveToPos(driveSys, shootingFromPosition)
            .onlyIf(
                () -> driveSys.getPosition().getTranslation().getDistance(returningShotPos) > 0.5)
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 2))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));

    autoChooser.addOption(
        "Shoot Pickup 2 drive shoot",
        AutoFactories.speaker(driveSys, arm, shooter)
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 2))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));

    autoChooser.addOption(
        "Shoot Pickup 1 drive shoot",
        AutoFactories.speaker(driveSys, arm, shooter)
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 1))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));
    autoChooser.addOption(
        "Shoot Pickup 0 drive shoot",
        AutoFactories.speaker(driveSys, arm, shooter)
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 0))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));

    autoChooser.addOption(
        "3 note (1 then 2)",
        AutoFactories.speaker(driveSys, arm, shooter)
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 1))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 2))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));

    autoChooser.addOption(
        "3 note (2 then 0)",
        AutoFactories.speaker(driveSys, arm, shooter)
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 2))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 0))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));

    autoChooser.addOption(
        "3 note (1 then 0)",
        AutoFactories.speaker(driveSys, arm, shooter)
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 1))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 0))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));

    autoChooser.addOption(
        "4 note (2 1 0)",
        AutoFactories.speaker(driveSys, arm, shooter)
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 2))
            .andThen(
                AutoFactories.driveToPos(
                    driveSys, new Pose2d(returningShotPos, Rotation2d.fromDegrees(-90))))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 1))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter))
            .andThen(AutoFactories.pickup(driveSys, arm, shooter, 0))
            .andThen(AutoFactories.driveToPos(driveSys, returningShotPos))
            .andThen(AutoFactories.speaker(driveSys, arm, shooter)));
  }

  /** Updates the pose estimator to use the correct initial pose */
  public void setPose(Pose2d pose) {
    driveSys.setPosition(pose);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    var teleopDrive = new TeleopDrive(driveSys, driverController);
    driveSys.setDefaultCommand(teleopDrive);

    // Intake commands
    driverController
        .leftTrigger(0.2)
        .whileTrue(
            Intake.fromFloor(shooter, arm, secondController.getHID())
                .andThen(
                    new StartEndCommand(
                            () -> driverController.getHID().setRumble(RumbleType.kLeftRumble, 1.0),
                            () -> driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.0))
                        .withTimeout(1.0)));
    secondController
        .leftBumper()
        .whileTrue(Intake.fromSource(shooter, arm, secondController.getHID()));
    secondController.b().whileTrue(Intake.fromFloor(shooter, arm, secondController.getHID()));

    LoggedTunableNumber ampPreset =
        new LoggedTunableNumber("Presets/Arm Amp", 0.17); // TODO: add value
    LoggedTunableNumber ampTop =
        new LoggedTunableNumber("Presets/Amp top", -3000); // TODO: add value
    LoggedTunableNumber ampBottom =
        new LoggedTunableNumber("Presets/Amp bottom", 500); // TODO: add value

    // Amp align
    secondController
        .povRight()
        .whileTrue(
            new RotateTo(driveSys, Rotation2d.fromDegrees(90))
                .andThen(
                    new HolonomicTrajectoryFollower(
                            driveSys,
                            () ->
                                AutoFactories.makeTrajectory(
                                    driveSys,
                                    new Pose2d(
                                        FieldConstants.getAmp(), Rotation2d.fromDegrees(90))),
                            Rotation2d.fromDegrees(90))
                        .repeatedly()
                        .until(
                            () ->
                                driveSys
                                            .getPosition()
                                            .getTranslation()
                                            .getDistance(FieldConstants.getAmp())
                                        < 0.085
                                    && Math.abs(
                                            driveSys
                                                .getPosition()
                                                .getRotation()
                                                .minus(Rotation2d.fromDegrees(90))
                                                .getDegrees())
                                        < 5))
                .raceWith(
                    Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(ampPreset.get()))))
                .andThen(
                    Commands.runOnce(() -> shooter.shoot(ampTop.get(), ampBottom.get()), shooter)));

    // Amp scoring
    secondController
        .leftTrigger(0.2)
        .whileTrue(
            Commands.run(
                () -> {
                  if (!Overrides.disableArmAnglePresets.get()) {
                    arm.setAngle(Rotation2d.fromRotations(ampPreset.get()));
                  } else {
                    arm.commandIncrement(
                        Rotation2d.fromRotations(
                            secondController.getLeftY()
                                * Overrides.armAngleOverrideIncrementScale.get()));
                  }
                },
                arm))
        .onFalse(Commands.runOnce(arm::commandNeutral, arm))
        .and(() -> secondController.getHID().getPOV() == 270)
        .onTrue(Commands.runOnce(() -> shooter.shoot(ampTop.get(), ampBottom.get()), shooter))
        .onFalse(Commands.runOnce(shooter::stop, shooter));

    // Speaker scoring
    driverController
        .rightTrigger(0.2)
        .and(
            new Trigger(Overrides.disableAutoAiming::get)
                .negate()
                .and(
                    new Trigger(Overrides.disableAutoAlign::get)
                        .negate())) // Make sure no overrides have been activated
        .whileTrue(new SpeakerShoot(driveSys, arm, teleopDrive::setCustomRotation, shooter));

    // Speaker scoring without auto-aim
    driverController
        .rightTrigger(0.2)
        .and(
            new Trigger(Overrides.disableAutoAiming::get)
                .or(Overrides.disableAutoAlign::get)) // Use this when overrides are activated
        .whileTrue(
            new SimpleSpeakerShoot(
                driveSys, arm, teleopDrive::setCustomRotation, shooter, secondController.getHID()));

    // Rapid eject
    secondController
        .a()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  shooter.setFeeder(-8000);
                  shooter.cmdVel(-2000, 2000);
                },
                () -> {
                  shooter.setFeeder(0);
                },
                shooter));

    // Climb
    climber.setDefaultCommand(
        new ClimbAndLevel(
            climber, () -> -secondController.getRightY(), () -> driveSys.getGyro().getX()));
    secondController
        .leftStick()
        .onTrue(
            new ClimbSplit(
                    climber,
                    () -> -secondController.getLeftY(),
                    () -> -secondController.getRightY())
                .until(() -> secondController.getHID().getRightStickButtonPressed()));
    // Pre-spin up
    secondController
        .rightTrigger(0.2)
        .whileTrue(Commands.run(() -> shooter.cmdVel(-4000, 4000), shooter));

    driverController.x().onTrue(Commands.runOnce(()->{driveSys.setPosition(          FieldConstants.flip(new Pose2d(1.4, 5.5, Rotation2d.fromRotations(0.5)))      );}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
