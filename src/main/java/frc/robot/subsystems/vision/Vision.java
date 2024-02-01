package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.utils.PeriodicRunnable;
import org.littletonrobotics.junction.Logger;

/** The vision subsystem */
public class Vision extends PeriodicRunnable {
  Transform3d[] camPoses;
  VisionIO[] ios;
  VisionIOInputs[] outs;
  Drivetrain drivetrain;

  /**
   * Constructs a <code>Vision</code> subsystem
   *
   * @param drivetrain The drivetrain to send the updates to
   * @param camPoses The positions of the cameras
   * @param ios The IOs of the cameras
   */
  public Vision(Drivetrain drivetrain, Transform3d[] camPoses, VisionIO[] ios) {
    super();
    this.camPoses = camPoses;
    this.ios = ios;
    assert ios.length == camPoses.length;
    this.outs = new VisionIOInputs[ios.length];
    for (int i = 0; i < ios.length; i++) {
      outs[i] = new VisionIOInputs();
      //      ios[i].updateInputs(outs[i]);
      //      Logger.processInputs("Vision/" + i, outs[i]);
    }
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(outs[i]);
      Logger.processInputs("Vision/" + i, outs[i]);
      if (outs[i].ids.isPresent() && outs[i].pose.isPresent() && outs[i].timestamp.isPresent()) {
        if (outs[i].ids.get().length > 0) {
          Pose3d outPose = new Pose3d();
          if (outs[i].ids.get().length == 1) {
            Rotation3d r1 =
                new Rotation3d(
                        new Quaternion(
                            outs[i].pose.get()[3],
                            outs[i].pose.get()[4],
                            outs[i].pose.get()[5],
                            outs[i].pose.get()[6]))
                    .rotateBy(new Rotation3d(0, 0, Math.PI));
            Pose3d tagpose =
                AprilTagFields.k2023ChargedUp
                    .loadAprilTagLayoutField()
                    .getTagPose(Integer.parseInt(outs[i].ids.get()[0]))
                    .orElse(new Pose3d());
            Rotation3d rot =
                new Rotation3d(
                    r1.getX(),
                    r1.getY(),
                    drivetrain
                        .getPosition()
                        .relativeTo(tagpose.toPose2d())
                        .getRotation()
                        .interpolate(r1.toRotation2d(), 0.5)
                        .getRadians());

            Transform3d as =
                new Transform3d(
                    new Translation3d(
                        outs[i].pose.get()[0], outs[i].pose.get()[1], outs[i].pose.get()[2]),
                    rot);
            outPose = tagpose.plus(as);
          } else if (outs[i].ids.get().length > 1) {
            outPose =
                new Pose3d(
                    outs[i].pose.get()[0],
                    outs[i].pose.get()[1],
                    outs[i].pose.get()[2],
                    new Rotation3d(
                        new Quaternion(
                            outs[i].pose.get()[3],
                            outs[i].pose.get()[4],
                            outs[i].pose.get()[5],
                            outs[i].pose.get()[6])));
          }
          Pose3d poseOfBot = outPose.plus(camPoses[i].inverse());
          Logger.recordOutput("Vision/EstPose_" + i, poseOfBot);
          drivetrain.addVisionUpdate(poseOfBot.toPose2d(), outs[i].timestamp.get());
        }
      }
    }
  }
}
