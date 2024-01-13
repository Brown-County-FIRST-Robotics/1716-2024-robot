# Standard CAN IDs for robot components

Each subsystem on the robot gets 25 IDs, with 0-10 being motors, 11-15 being non-motor output, 16-20 being input, and 21-25 being miscellaneous. There are breaks in the table between each section.

## Drivetrain

|Component|ID|
|-|-|
|Front Left Motor|0|
|Front Right Motor|1|
|Back Left Motor|2|
|Back Right Motor|3|
|||
|Pigeon|16|

## Arm

|Component|ID|
|-|-|
|||

## Solenoids

Solenoids have their own separate ID system off of the Pneumatic Hub, ranging from 1-16. Double solenoids have two IDs, one for forward and one for reverse.

|Solenoid|ID(s)|
|-|-|
|||
