// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.mecanum.MecanumDrivetrain;
import frc.robot.subsystems.mecanum.MecanumIO;
import frc.robot.subsystems.mecanum.MecanumIOSpark;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSparkFX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOSecondSight;
import frc.robot.utils.HolonomicTrajectoryFollower;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;
import frc.robot.utils.ShootWhileMove;

import java.util.List;

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
                  new ModuleIOSim(0),
                  new ModuleIOSim(1),
                  new ModuleIOSim(2),
                  new ModuleIOSim(3),
                  new IMUIOSim());
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new ModuleIOSparkFX(22, 10, "FL"),
                  new ModuleIOSparkFX(23, 12, "FR"),
                  new ModuleIOSparkFX(21, 13, "BL"),
                  new ModuleIOSparkFX(20, 11, "BR"),
                  new IMUIONavx());
          var vision =
              new Vision(
                  driveSys,
                  new Transform3d[] {
                    new Transform3d(
                        new Translation3d(5 * 0.0254, 2 * 0.0254, 22 * 0.0254),
                        new Rotation3d(0, -12 * Math.PI / 180, 0))
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
            arm = new Arm(new ArmIOSparkFlex(55));
            break;
          case SHOOTER:
            shooter = new Shooter(new ShooterIOSparkFlexes(58, 57), new FeederIOSpark550(41, 0, 1));
            break;
        }
      }
    } else {
      switch (WhoAmI.bot) {
        case SIMSWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new IMUIO() {});
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new IMUIO() {});
          var vision =
              new Vision(
                  driveSys,
                  new Transform3d[] {
                    new Transform3d(
                        new Translation3d(3 * 0.0254, 10 * 0.0254, 22 * 0.0254),
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
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {}, new FeederIO() {});
    }

    useAlliance();
    configureBindings();
  }

  /** Updates the pose estimator to use the correct initial pose */
  public void useAlliance() {
    driveSys.setPosition(
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue
            ? Constants.BLUE_INIT_POSE
            : Constants.RED_INIT_POSE);
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

    LoggedTunableNumber ampPreset =
        new LoggedTunableNumber("Presets/Arm Amp", 0.15); // TODO: add value
    LoggedTunableNumber ampTop =
        new LoggedTunableNumber("Presets/Amp top", -2000); // TODO: add value
    LoggedTunableNumber ampBottom =
        new LoggedTunableNumber("Presets/Amp bottom", 2000); // TODO: add value

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
        .whileTrue(
            new SpeakerShoot(
                driveSys, arm, teleopDrive::setCustomRotation, shooter, secondController.getHID()));

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
                  shooter.cmdVel(-1000, 1000);
                },
                () -> {
                  shooter.setFeeder(0);
                },
                shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    var rt=new RotateTo(driveSys);
    return new SpeakerShoot(driveSys,arm,rt::setCustomRotation,shooter, secondController.getHID()).alongWith(rt).andThen(new RotateTo(driveSys,Rotation2d.fromRotations(0.5))).andThen(new HolonomicTrajectoryFollower(driveSys,()->{
      var conf=new TrajectoryConfig(3,1);
      var target=FieldConstants.getGamePiece(1);
      var startRot=driveSys.getPosition().getTranslation().minus(target).getAngle();
      var startSpeed= ShootWhileMove.getFieldRelativeSpeeds(driveSys.getVelocity(),driveSys.getPosition().getRotation());
      if(startSpeed.getNorm()>0.1){
        startRot=startSpeed.getAngle();
        conf=conf.setStartVelocity(startSpeed.getNorm());
      }
      return TrajectoryGenerator.generateTrajectory(new Pose2d(driveSys.getPosition().getTranslation(),startRot), List.of(),new Pose2d(target,new Rotation2d()),conf);
    },Rotation2d.fromRotations(0.5)).alongWith(Intake.fromFloor(shooter,arm,secondController.getHID())));
  }
}
