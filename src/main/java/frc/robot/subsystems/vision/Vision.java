package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.utils.PeriodicRunnable;
import org.littletonrobotics.junction.Logger;

public class Vision extends PeriodicRunnable {
  Transform3d[] camPoses;
  VisionIO[] ios;
  VisionIOInputs[] outs;
  Drivetrain drivetrain;

  public Vision(Drivetrain drivetrain, Transform3d[] camPoses, VisionIO[] ios) {
    super();
    this.camPoses = camPoses;
    this.ios = ios;
    assert ios.length == camPoses.length;
    this.outs = new VisionIOInputs[ios.length];
    for (int i = 0; i < ios.length; i++) {
      outs[i] = new VisionIOInputs();
      ios[i].updateInputs(outs[i]);
      Logger.processInputs("Vision/" + i, outs[i]);
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
                        outs[i].poses[j][6]));
//            Rotation3d rot =
//                new Rotation3d(
//                    r1.getX(),
//                    r1.getY(),
//                    drivetrain
//                        .getPosition()
//                        .getRotation()
//                        .interpolate(r1.toRotation2d(), 1)
//                        .getRadians());
            Transform3d as =
                new Transform3d(
                    new Translation3d(
                        outs[i].poses[j][0], outs[i].poses[j][1], outs[i].poses[j][2]),
                    r1.unaryMinus());
            Pose3d tagpose =
                AprilTagFields.k2023ChargedUp
                    .loadAprilTagLayoutField()
                    .getTagPose(Integer.parseInt(outs[i].ids[j][0]))
                    .orElse(new Pose3d());
            outPose = tagpose.plus(as.inverse());
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
