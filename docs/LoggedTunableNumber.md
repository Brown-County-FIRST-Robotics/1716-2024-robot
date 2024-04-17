# Logged Tunable Number
`LoggedTunableNumber` is a wrapper around a numeric dashboard input.


For Spark Max/Spark Flex
```
SparkPIDController pid;

LoggedTunableNumber pTuner=new LoggedTunableNumber("Arm/P");
pTuner.initDefault(1.0); // Sets the default
pTuner.attach(pid::setP); // Attaches listener to input, and calls lambda with default value
```

For WPILib PID
```
PIDController pid;

LoggedTunableNumber pTuner=new LoggedTunableNumber("Arm/P");
pTuner.initDefault(1.0); // Sets the default
pTuner.attach(pid::setP); // Attaches listener to input, and calls lambda with default value
```

For TalonFX
```
TalonFX motor;

LoggedTunableNumber pTuner=new LoggedTunableNumber("Arm/P");
pTuner.initDefault(1.0); // Sets the default
pTuner.attach((Double v)->{
    TalonFXConfiguration config = new TalonFXConfiguration();
    thrust.getConfigurator().refresh(config);
    config.Slot0.kP=v;
    thrust.getConfigurator().apply(config);
}); // Attaches listener to input, and calls lambda with default value
```
