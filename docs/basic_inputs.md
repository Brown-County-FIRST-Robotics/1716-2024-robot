# Basic inputs for a flywheel

position, velocity, current, applied output, temperature, closed loop error (falcons only)

log firmware

## REV example:
FlywheelIO.java
```java
package frc.robot.subysytems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs{
    double position = 0.0;
    double velocity = 0.0;
    double current = 0.0;
    double output = 0.0;
    double temperature=0.0;
  }
  default void updateInputs(FlywheelIOInputs inputs) {}
  
  default void commandVelocity(double vel) {}
}
```

FlywheelIOSparkFlex.java
```java
package frc.robot.subysytems.flywheel;

import com.revrobotics.*;
import frc.robot.Constants;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.LoggedTunableNumber;

public class FlywheelIOSparkFlex implements FlywheelIO {
  CANSparkMax controller;
  SparkPIDController pid;
  RelativeEncoder encoder;
  private static final double FREE_RPM = 5676.0;
  LoggedTunableNumber ffTuner = new LoggedTunableNumber("Flywheel/ff_tuner", 1.0 / FREE_RPM);
  LoggedTunableNumber pTuner = new LoggedTunableNumber("Flywheel/p_tuner", 1.0 / FREE_RPM);
  LoggedTunableNumber iTuner = new LoggedTunableNumber("Flywheel/i_tuner", 0.0);
  LoggedTunableNumber dTuner = new LoggedTunableNumber("Flywheel/d_tuner", 0.0);
  
  public FlywheelIOSparkFlex(int id) {
    controller = new CANSparkFlex(id, CANSparkLowLevel.MotorType.kBrushless);
    pid = controller.getPIDController();
    encoder=controller.getEncoder();
    controller.restoreFactoryDefaults();
    controller.setIdleMode(CANSparkBase.IdleMode.kBrake);
    controller.setSmartCurrentLimit(Constants.CurrentLimits.NEO_VORTEX);
    pid.setFeedbackDevice(encoder);
    pid.setOutputRange(-1, 1);
    pid.setSmartMotionMaxVelocity(FREE_RPM, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    ffTuner.attach(pid::setFF);
    pTuner.attach(pid::setP);
    iTuner.attach(pid::setI);
    dTuner.attach(pid::setD);
    controller.burnFlash();
    CustomAlerts.makeOverTempAlert(controller, 60, 50, "Flywheel motor");
    Logger.recordOutput("Firmware/FlywheelController", controller.getFirmwareString());
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();
    inputs.current = controller.getOutputCurrent();
    inputs.output = controller.getAppliedOutput();
    inputs.temperature = controller.getMotorTemperature();
  }
  
  @Override
  public void commandVelocity(double vel) {
    pid.setReference(vel, CANSparkBase.ControlType.kVelocity, 0);
  }
}
```

Flywheel.java
```java
package frc.robot.subysytems.flywheel;

public class FlywheelIOSparkFlex implements FlywheelIO {

```