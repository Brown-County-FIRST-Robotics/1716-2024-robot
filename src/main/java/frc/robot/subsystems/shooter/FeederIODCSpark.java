package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.Encoder;

public class FeederIODCSpark implements FeederIO {
  CANSparkMax controller;
  Encoder encoder;

  public FeederIODCSpark(int id, int encA, int encB) {
    controller = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushed);
    controller.restoreFactoryDefaults();
    controller.setSmartCurrentLimit(20);
    controller.setIdleMode(CANSparkBase.IdleMode.kCoast);
    controller.setInverted(true);
    controller.burnFlash();
    encoder = new Encoder(encA, encB);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.position = encoder.get();
    inputs.velocity = encoder.getRate();
    inputs.current = controller.getOutputCurrent();
  }

  @Override
  public void setVoltage(double voltage) {
    controller.setVoltage(voltage);
  }

  @Override
  public void resetPos() {
    encoder.reset();
  }
}
