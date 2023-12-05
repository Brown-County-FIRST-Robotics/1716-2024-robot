// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMUIONavx;
import frc.robot.subsystems.mecanum.MecanumDrivetrain;
import frc.robot.subsystems.mecanum.MecanumIOSpark;
import org.littletonrobotics.junction.Logger;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final Drivetrain driveSys;
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (WhoAmI.bot){
            case MECHBASE:
                driveSys=new MecanumDrivetrain(new MecanumIOSpark(1,2,3,4),new IMUIONavx());
                break;
            default:
                driveSys=new MecanumDrivetrain(new MecanumIOSpark(1,2,3,4),new IMUIONavx());
        }

        driveSys.setDefaultCommand(new TeleopDrive(driveSys, driverController));
        // Configure the trigger bindings
        configureBindings();
    }

    private void postGitData(){
        File deployDir = Filesystem.getDeployDirectory();
        File hashFile = new File(deployDir, "git_hash.txt");
        File statusFile = new File(deployDir, "git_status.txt");
        File deployerFile = new File(deployDir, "deployer.txt");
        String hash;
        String status="";
        String deployer;
        try {
            Scanner reader = new Scanner(hashFile);
            hash = reader.nextLine();
            reader.close();
        } catch (FileNotFoundException e) {
            hash="Deploy did not send git data";
        }
        try {
            Scanner reader = new Scanner(statusFile);
            while(reader.hasNext()){
                status += reader.nextLine();
            }
            reader.close();
        } catch (FileNotFoundException e) {
            status="Deploy did not send git data";
        }
        try {
            Scanner reader = new Scanner(deployerFile);
            deployer = reader.nextLine();
            reader.close();
        } catch (FileNotFoundException e) {
            deployer="Unknown deployer";
        }
        Logger.recordMetadata("Commit Hash", hash);
        Logger.recordMetadata("Git Status", status);
        Logger.recordMetadata("Deployer", deployer);
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return null;
    }
}
