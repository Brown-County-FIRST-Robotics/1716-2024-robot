package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;

public class Intake extends Command {
  Shooter shooter;
  Arm arm;
  XboxController controller;
  LoggedTunableNumber armPreset = new LoggedTunableNumber("Presets/Intake_Arm", -0.2);
  LoggedTunableNumber topSpeed = new LoggedTunableNumber("Presets/Intake_Top", 1700);
  LoggedTunableNumber bottomSpeed = new LoggedTunableNumber("Presets/Intake_Bottom", -2000);

  public Intake(Shooter shooter, Arm arm, XboxController overrideController) {
    this.shooter = shooter;
    this.arm = arm;
    controller = overrideController;
    addRequirements(shooter, arm);
  }

  @Override
  public void initialize() {
    shooter.intaking=true;
    setSpeedsAndPositions();
    shooter.cmdFeeder(8000);
  }

  @Override
  public void execute() {
    setSpeedsAndPositions();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.cmdVel(0, 0);
    shooter.cmdFeeder(0);
    arm.commandNeutral();
  }

  @Override
  public boolean isFinished() {
    return shooter.isHolding();
  }

  private void setSpeedsAndPositions() {
    if (!Overrides.disableArmAnglePresets.get()) {
      arm.setAngle(Rotation2d.fromRotations(armPreset.get()));
    } else {
      arm.commandIncrement(
          Rotation2d.fromRotations(
              controller.getLeftY() * Overrides.armAngleOverrideIncrementScale.get()));
    }
    shooter.cmdVel(topSpeed.get(), bottomSpeed.get());
  }
}
