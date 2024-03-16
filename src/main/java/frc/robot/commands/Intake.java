package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;

/**
 * A command to Intake. Create using factory methods {@link Intake#fromFloor} and {@link
 * Intake#fromSource} for floor and source respectively.
 */
public class Intake extends Command {
  Shooter shooter;
  Arm arm;
  XboxController controller;
  static LoggedTunableNumber armPreset = new LoggedTunableNumber("Presets/Intake_Arm", -0.16);
  static LoggedTunableNumber topSpeed = new LoggedTunableNumber("Presets/Intake_Top", 1700);
  static LoggedTunableNumber bottomSpeed = new LoggedTunableNumber("Presets/Intake_Bottom", -2000);
  static LoggedTunableNumber armSourcePreset =
      new LoggedTunableNumber("Presets/Intake_Arm_Source", 0.16); // TODO: find real preset
  LoggedTunableNumber preset;

  /**
   * Makes a new command to intake from the floor
   *
   * @param shooter The shooter subsystem
   * @param arm The arm subsystem
   * @param overrideController The override controller
   * @return A new command that intakes from the floor
   */
  public static Intake fromFloor(Shooter shooter, Arm arm, XboxController overrideController) {
    return new Intake(shooter, arm, overrideController, armPreset);
  }

  public static Intake fromFloor(Shooter shooter, Arm arm) {
    return new Intake(shooter, arm, armPreset);
  }

  /**
   * Makes a new command to intake from the source
   *
   * @param shooter The shooter subsystem
   * @param arm The arm subsystem
   * @param overrideController The override controller
   * @return A new command that intakes from the source
   */
  public static Intake fromSource(Shooter shooter, Arm arm, XboxController overrideController) {
    return new Intake(shooter, arm, overrideController, armSourcePreset);
  }

  private Intake(Shooter shooter, Arm arm, LoggedTunableNumber preset) {
    this.shooter = shooter;
    this.preset = preset;
    this.arm = arm;
    addRequirements(shooter, arm);
  }

  private Intake(
      Shooter shooter, Arm arm, XboxController overrideController, LoggedTunableNumber preset) {
    this(shooter, arm, preset);
    controller = overrideController;
  }

  @Override
  public void initialize() {
    shooter.intaking = true;
    shooter.setHolding(false);
    setSpeedsAndPositions();
    shooter.setFeeder(8000);
  }

  @Override
  public void execute() {
    setSpeedsAndPositions();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.cmdVel(0, 0);
    shooter.setFeeder(0);
    arm.commandNeutral();
    if(!interrupted){
      LEDs.getInstance().mode1();
      LEDs.getInstance().intakelight();
    }
  }

  @Override
  public boolean isFinished() {
    return shooter.isHolding();
  }

  private void setSpeedsAndPositions() {
    if (controller != null) {
      if (!Overrides.disableArmAnglePresets.get() && !controller.getBButton()) {
        arm.setAngle(Rotation2d.fromRotations(preset.get()));
      } else if (Overrides.disableArmAnglePresets.get()) {
        arm.commandIncrement(
            Rotation2d.fromRotations(
                controller.getLeftY() * Overrides.armAngleOverrideIncrementScale.get()));
      }
    } else {
      arm.setAngle(Rotation2d.fromRotations(preset.get()));
    }
    shooter.cmdVel(topSpeed.get(), bottomSpeed.get());
  }
}
