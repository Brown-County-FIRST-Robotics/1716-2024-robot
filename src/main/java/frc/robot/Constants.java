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
  public static class OperatorConstants {
    /** The port the driver controller is on */
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class Auto {
    public static final double X_P = 0;
    public static final double X_I = 0;
    public static final double X_D = 0;

    public static final double Y_P = 0;
    public static final double Y_I = 0;
    public static final double Y_D = 0;

    public static final double THETA_P = 0;
    public static final double THETA_I = 0;
    public static final double THETA_D = 0;

    public static final double MAX_ANGULAR_VELOCITY = 1;
    public static final double MAX_ANGULAR_ACCELERATION = 1;
    public static final double MAX_VELOCITY = 5;
    public static final double MAX_ACCELERATION = 3;
  }

  public static final class Driver {
    public static final double MAX_X_SPEED = 1;
    public static final double MAX_Y_SPEED = 1;
    public static final double MAX_THETA_SPEED = 6;
  }

  public static final Pose2d INIT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
  public static final Pose2d RED_INIT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static final Pose2d BLUE_INIT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
}
