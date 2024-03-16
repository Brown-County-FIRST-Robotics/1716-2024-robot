// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;

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
                  new Module(new ModuleIOSparkFX(23, 12, "FR"), 1),
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
        .whileTrue(Intake.fromFloor(shooter, arm, secondController.getHID()));
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
                () -> shooter.setFeeder(0),
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // 3 note auto
    // Shoot into speaker, pickup 7 (fallback on 6), drive closer to speaker ,shoot into speaker, pickup 2, shoot into speaker
    return AutoFactories.speaker(driveSys, arm, shooter)
        .andThen(AutoFactories.pickupWithBackup(driveSys, arm, shooter, 7, 6))
        .andThen(
            AutoFactories.driveToPos( // Drives to about 3 feet behind piece 2
                driveSys, FieldConstants.getGamePiece(2).plus(new Translation2d(1, 0))))
        .andThen(AutoFactories.speaker(driveSys, arm, shooter))
        .andThen(AutoFactories.pickup(driveSys, arm, shooter, 2))
        .andThen(AutoFactories.speaker(driveSys, arm, shooter));
  }
}
