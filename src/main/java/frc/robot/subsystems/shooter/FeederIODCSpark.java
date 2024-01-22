package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.shooter.FeederIO.*;
import frc.robot.utils.LoggedTunableNumber;

public class FeederIODCSpark implements FeederIO {
  CANSparkMax controller;
  Encoder encoder;
  PIDController pid = new PIDController(1, 0, 0);
  LoggedTunableNumber p = new LoggedTunableNumber("feed_p", 1);
  LoggedTunableNumber i = new LoggedTunableNumber("feed_i", 0);
  LoggedTunableNumber d = new LoggedTunableNumber("feed_d", 0);

  public FeederIODCSpark(int id, int encA, int encB) {
    controller = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushed);
    encoder = new Encoder(encA, encB);
    p.attach(pid::setP);
    i.attach(pid::setI);
    d.attach(pid::setD);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederPos = encoder.get();
  }

  @Override
  public void cmdPos(double pos) {
    controller.set(pid.calculate(encoder.get(), pos));
  }

  @Override
  public void resetPos() {
    encoder.reset();
  }
}
