package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** The IO layer for one SecondSight camera */
public class VisionIOSecondSight implements VisionIO {
  BooleanSubscriber isRecordingSub;
  StringSubscriber recordingPathSub;
  StringArraySubscriber idsSub;
  double lastTime = 0.0;
  DoubleArraySubscriber posesSub;
  DoubleSubscriber errorSub;

  PhotonCamera cam = new PhotonCamera("Arducam_OV2311_USB_Camera (1)");
  Transform3d robotToCam =
      new Transform3d(
          new Translation3d(2 * 0.0254, 0 * 0.0254, 24 * 0.0254),
          new Rotation3d(
              0,
              -25.0 * Math.PI / 180,
              0.0*Math.PI / 180));

  // Construct PhotonPoseEstimator
  PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
          PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          cam,
          robotToCam);

  /**
   * Constructs a new <code>VisionIOSecondSight</code> from a NT path
   *
   * @param inst_name The NT path of the instance
   * @param cam_name The name of the camera
   */
  public VisionIOSecondSight(String inst_name, String cam_name) {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var k = photonPoseEstimator.update();
    if (k.isPresent()) {
      inputs.pose = Optional.ofNullable(k.get().estimatedPose);
      inputs.timestamp = Optional.of(k.get().timestampSeconds);
    } else {
      inputs.pose = Optional.empty();
      inputs.timestamp = Optional.empty();
    }
  }
}
