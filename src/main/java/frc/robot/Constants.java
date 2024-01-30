// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Constants for autonomous operation */
  public static final class Auto {
    /** P value for the trajectory x PID loop */
    public static final double X_P = 0;
    /** I value for the trajectory x PID loop */
    public static final double X_I = 0;
    /** D value for the trajectory x PID loop */
    public static final double X_D = 0;
    /** P value for the trajectory y PID loop */
    public static final double Y_P = 0;
    /** I value for the trajectory y PID loop */
    public static final double Y_I = 0;
    /** D value for the trajectory y PID loop */
    public static final double Y_D = 0;
    /** P value for the trajectory theta PID loop */
    public static final double THETA_P = 0;
    /** I value for the trajectory theta PID loop */
    public static final double THETA_I = 0;
    /** D value for the trajectory theta PID loop */
    public static final double THETA_D = 0;

    /** The max angular velocity value in a trajectory feedback loop */
    public static final double MAX_ANGULAR_VELOCITY = 1;
    /** The max angular acceleration value in a trajectory feedback loop */
    public static final double MAX_ANGULAR_ACCELERATION = 1;
    /** The max velocity for trajectories */
    public static final double MAX_VELOCITY = 5;
    /** The max acceleration for trajectories */
    public static final double MAX_ACCELERATION = 3;
  }

  /** Constants relating to manual operation */
  public static final class Driver {
    /** The maximum x velocity during manual operation */
    public static final double MAX_X_SPEED = 4;
    /** The maximum y velocity during manual operation */
    public static final double MAX_Y_SPEED = 4;
    /** The maximum angular velocity during manual operation */
    public static final double MAX_THETA_SPEED = 6;
    /** The port the driver controller is on */
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  /** Current limits for different motors */
  public static final class CurrentLimits {
    /** Current limit for NEO 550 */
    public static final int NEO550 = 20;
    /** Current limit for NEO v1.1 */
    public static final int NEO = 50;
    /** Current limit for NEO vortex */
    public static final int NEO_VORTEX = 80;
  }

  /** The initial pose of the robot */
  public static final Pose2d INIT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
  /** The initial pose of the robot when it is on the red alliance */
  public static final Pose2d RED_INIT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  /** The initial pose of the robot when it is on the blue alliance */
  public static final Pose2d BLUE_INIT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
}
