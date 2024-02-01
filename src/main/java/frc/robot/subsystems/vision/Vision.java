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
      for (int j = 0; j < outs[i].ids.length; j++) {
        if (outs[i].ids[j].length > 0) {
          Pose3d outPose = new Pose3d();
          if (outs[i].ids[j].length == 1) {
            Rotation3d r1 =
                new Rotation3d(
                        new Quaternion(
                            outs[i].poses[j][3],
                            outs[i].poses[j][4],
                            outs[i].poses[j][5],
                            outs[i].poses[j][6]))
                    .rotateBy(new Rotation3d(0, 0, Math.PI));
            Pose3d tagpose =
                AprilTagFields.k2023ChargedUp
                    .loadAprilTagLayoutField()
                    .getTagPose(Integer.parseInt(outs[i].ids[j][0]))
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
                        outs[i].poses[j][0], outs[i].poses[j][1], outs[i].poses[j][2]),
                    rot);
            outPose = tagpose.plus(as);
          } else if (outs[i].ids[j].length > 1) {
            outPose =
                new Pose3d(
                    outs[i].poses[j][0],
                    outs[i].poses[j][1],
                    outs[i].poses[j][2],
                    new Rotation3d(
                        new Quaternion(
                            outs[i].poses[j][3],
                            outs[i].poses[j][4],
                            outs[i].poses[j][5],
                            outs[i].poses[j][6])));
          }
          Pose3d poseOfBot = outPose.plus(camPoses[i].inverse());
          Logger.recordOutput("Vision/EstPose_" + i + "_" + j, poseOfBot);
          drivetrain.addVisionUpdate(poseOfBot.toPose2d(), outs[i].timestamps[j]);
        }
      }
    }
  }
}
