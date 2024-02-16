package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
  VisionIOInputs[] inputs;
  Drivetrain drivetrain;
  AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

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
    if (camPoses.length != ios.length) {
      throw new IllegalArgumentException("Number of IOs and camera poses do not match");
    }
    this.inputs = new VisionIOInputs[ios.length];
    for (int i = 0; i < ios.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.length; i++) {
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Inputs_" + i, inputs[i]);
      if (inputs[i].ids.isPresent()
          && inputs[i].pose.isPresent()
          && inputs[i].timestamp.isPresent()) {
        if (inputs[i].ids.get().length > 0) {
          Pose3d outPose = new Pose3d();
          if (inputs[i].ids.get().length == 1) {
            Rotation3d r1 =
                new Rotation3d(
                        new Quaternion(
                            inputs[i].pose.get()[3],
                            inputs[i].pose.get()[4],
                            inputs[i].pose.get()[5],
                            inputs[i].pose.get()[6]))
                    .rotateBy(new Rotation3d(0, 0, Math.PI));
            Pose3d tagpose =
                layout.getTagPose(Integer.parseInt(inputs[i].ids.get()[0])).orElse(new Pose3d());
            Rotation3d rot =
                new Rotation3d(
                    r1.getX(),
                    r1.getY(),
                    drivetrain
                        .getPosition()
                        .relativeTo(tagpose.toPose2d())
                        .getRotation()
                        .interpolate(r1.toRotation2d(), 0.02)
                        .getRadians());

            Transform3d as =
                new Transform3d(
                    new Translation3d(
                        inputs[i].pose.get()[0], inputs[i].pose.get()[1], inputs[i].pose.get()[2]),
                    rot);
            outPose = tagpose.plus(as);
          } else if (inputs[i].ids.get().length > 1) {
            outPose =
                new Pose3d(
                    inputs[i].pose.get()[0],
                    inputs[i].pose.get()[1],
                    inputs[i].pose.get()[2],
                    new Rotation3d(
                        new Quaternion(
                            inputs[i].pose.get()[3],
                            inputs[i].pose.get()[4],
                            inputs[i].pose.get()[5],
                            inputs[i].pose.get()[6])));
          }
          Pose3d poseOfBot = outPose.plus(camPoses[i].inverse());
          Logger.recordOutput("Vision/EstPose_" + i, poseOfBot);
          drivetrain.addVisionUpdate(poseOfBot.toPose2d(), inputs[i].timestamp.get());
        }
      }
    }
  }
}
