package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;

public class ClimberIOSparkMaxes implements ClimberIO {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    public ClimberIOSparkMaxes(int leftID, int rightID) {
        leftMotor = new CANSparkMax(leftID, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightID, CANSparkMax.MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(50);
        rightMotor.setSmartCurrentLimit(50);

        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.leftPosition = leftMotor.getEncoder().getPosition();
        inputs.rightPosition = rightMotor.getEncoder().getPosition();

        inputs.leftSpeed = leftMotor.getEncoder().getVelocity();
        inputs.rightSpeed = rightMotor.getEncoder().getVelocity();

        inputs.leftTemp = leftMotor.getMotorTemperature();
        inputs.rightTemp = rightMotor.getMotorTemperature();

        inputs.leftOut = leftMotor.getAppliedOutput();
        inputs.rightOut = rightMotor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        leftMotor.setVoltage(leftVoltage);
        rightMotor.setVoltage(rightVoltage);
    }
}