# REV Motor Control
Recommended reading: command_based.md, motor_control_shared.md



All code examples in this document should not be run directly
## Feedback sensors
REV supports limit switches, duty cycle encoders, and analog encoders. REV also supports adding relative encoders to brushed motors, via alternate encoder mode. 


The builtin encoder can be accessed using `CANSparkBase.getEncoder()`

