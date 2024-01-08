# WPILib Reference Document

## Table of Contents:

1. [Component Class Reference](#component-class-reference)
2. [3rd Party Vendor Libraries](#3rd-party-vendor-libraries)
3. [Posting Dashboard Values](#posting-dashboard-values) 
4. [Posting NetworkTables Values]()
5. [Solenoids]()

## Component Class Reference:

|Class|Import Path|Notes|
|-|-|-|
|`TalonFX`|`com.ctre.phoenix6.hardware.TalonFX`|-|
|`CANSparkMax`|`com.revrobotics.CANSparkMax`|-|
|`TalonSRX`|`com.ctre.phoenix6.hardware.TalonSRX`|This is probably right, unconfirmed|
|`DoubleSolenoid`|`edu.wpi.first.wpilibj.DoubleSolenoid`|Positions are accessible at `DoubleSolenoid.Value`; more info available [here](#solenoids)|

## 3rd Party Vendor Libraries:

*3rd Party Vendor Libraries* are libraries written and distributed by component manufacturers for use in interfacing with their components (usually motors). To install REVLib or the CTRE Phoenix library:

1. Install their respective software. 
	- For REVLib, download the [Java/C++ API](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#c++-and-java) (big orange button with "Download Latest JAVA API" on it) and unzip into the C:\Users\Public\wpilib\2023 directory on Windows or ~/wpilib/2023 directory on Linux, as described on the website under offline installation. 
	- For the CTRE Phoenix library, download and install the [Phoenix Framework](https://store.ctr-electronics.com/software/).
2. Next, in VSCode, press the wpilib icon in the top right in your project and go to `WPILib: Manage Vendor Libraries` then `Install new libraries (offline)` and select `REVLib`, `Phoenix (v5)`, or both.

To uninstall a library, go to `WPILib: Manage Vendor Libraries` again and select `Manage current libraries`, then select any libraries you wish to remove and press enter. This menu can also be used to check what libraries you currently have installed.

> ***IMPORTANT:*** **Do not** install the `Phoenix (Pro)` library, as it is locked behind a paywall and will prevent your code from building while installed.

## Posting Dashboard Values:

FILL THIS IN COLIN!

## Posting NetworkTables Values:

Another feature of WPILib is NetworkTables, which can be used to communicate values between the driverstation computer, the RoboRIO, and any coprocessors (such as Raspberry Pi's) that the robot my have on it. All NetworkTable values are copied to all devices connected to NetworkTables, and values are organized similarly to a filesystem, where the folders are "subtables" and the files are "topics" (the NetworkTables term for a value). You can reference a topic/value by accessing its table, similarly to opening a folder, then declaring a `subscriber` to read topics/values, or a `publisher` to write topics/values.
