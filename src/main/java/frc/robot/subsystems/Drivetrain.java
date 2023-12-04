package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;

public interface Drivetrain extends Subsystem {
    Pose2d getPosition();
    void setPosition(Pose2d newPose);
    Command getDriveToPointCmd(Pose2d pose);
    Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY);
    Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose);
    Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY);
    void humanDrive(ChassisSpeeds cmd, boolean foc);

    Rotation3d getGyro();
    /**
     * Gets the acceleration values from the IMU
     * @return Array of accelerations (in MPS^2) as [x,y,z]
     */
    double[] getAcceleration();
}
