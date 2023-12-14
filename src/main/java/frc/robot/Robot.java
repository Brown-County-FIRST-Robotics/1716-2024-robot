package frc.robot;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.FileNotFoundException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Scanner;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.getInstance().recordMetadata("ProjectName", "2024");
    File deployDir = Filesystem.getDeployDirectory();
    File tagFile = new File(deployDir, "git_tag.txt");
    File deployerFile = new File(deployDir, "deployer.txt");
    String tagName;
    String deployer;
    try {
      Scanner reader = new Scanner(tagFile);
      tagName = reader.nextLine();
      reader.close();
    } catch (FileNotFoundException e) {
      tagName = "Deploy did not send git data";
    }
    try {
      Scanner reader = new Scanner(deployerFile);
      deployer = reader.nextLine();
      reader.close();
    } catch (FileNotFoundException e) {
      deployer = "Unknown deployer";
    }
    Logger.getInstance().recordMetadata("Tag Name", tagName);
    Logger.getInstance().recordMetadata("Deployer", deployer);
    Logger.getInstance().recordMetadata("Bot", String.valueOf(WhoAmI.bot));
    Logger.getInstance().recordMetadata("SN", HALUtil.getSerialNumber());

    // Running on a real robot, log to a USB stick
    switch (WhoAmI.mode) {
      case REAL:
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser/"));
        break;
      case SIM:
        Logger.getInstance().addDataReceiver(new WPILOGWriter("SimLogs/"));
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
        Logger.getInstance()
            .addDataReceiver(
                new WPILOGWriter(
                    LogFileUtil.addPathSuffix(
                        logPath,
                        "_replay" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()))));
        break;
    }
    Logger.getInstance().addDataReceiver(new NT4Publisher());

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.getInstance().start();
    robotContainer = new RobotContainer();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  int ct = 0;

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (ct == 3) {
      robotContainer.useAlliance();
    }
    ct += 1;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SwerveSimManager.getInstance().propagate();
  }
}
