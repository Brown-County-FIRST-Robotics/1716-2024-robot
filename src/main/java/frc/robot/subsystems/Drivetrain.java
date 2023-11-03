package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

public interface Drivetrain {
    Pose2d getPosition();
    void setPosition(Pose2d newPose);
    Command getDriveToPointCmd(Pose2d pose);
    Command getDriveToPointCmd(Pose2d pose, double endVelX, double endVelY);
    Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose);
    Command getFollowWaypointsCmd(List<Translation2d> waypoints, Pose2d pose, double endVelX, double endVelY);
    void humanDrive(double xVel, double yVel, double thetaVel, boolean fieldOriented);
    // TODO: add some stuff for getting gyro values
}
