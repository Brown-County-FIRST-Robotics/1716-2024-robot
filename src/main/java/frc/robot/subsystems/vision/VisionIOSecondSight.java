package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import java.util.Optional;

/** The IO layer for one SecondSight camera */
public class VisionIOSecondSight implements VisionIO {
  BooleanSubscriber isRecordingSub;
  StringSubscriber recordingPathSub;
  StringArraySubscriber idsSub;
  double lastTime = 0.0;
  DoubleArraySubscriber posesSub;
  DoubleSubscriber errorSub;

  /**
   * Constructs a new <code>VisionIOSecondSight</code> from a NT path
   *
   * @param name The NT path of the camera
   */
  public VisionIOSecondSight(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SS_LAPTOP").getSubTable("0");
    isRecordingSub = table.getBooleanTopic("isRecording").subscribe(false);
    recordingPathSub = table.getStringTopic("recordingPath").subscribe("");
    idsSub =
        table
            .getStringArrayTopic("IDs")
            .subscribe(
                new String[] {"asd"},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(3));
    posesSub =
        table
            .getDoubleArrayTopic("Pose")
            .subscribe(
                new double[] {0, 0, 0, 0, 0, 0, 0},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(3));
    errorSub =
        table
            .getDoubleTopic("RMSError")
            .subscribe(
                -1.0,
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(3));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isRecording = isRecordingSub.get();
    inputs.recordingPath = recordingPathSub.get();
    double timestamp = idsSub.getLastChange() / 1000000.0;
    if (timestamp > lastTime) {
      lastTime = timestamp;
      inputs.pose = Optional.of(posesSub.get());
      inputs.ids = Optional.of(idsSub.get());
      inputs.errors = Optional.of(errorSub.get());
      inputs.timestamp = Optional.of(timestamp);
    } else {
      inputs.pose = Optional.empty();
      inputs.ids = Optional.empty();
      inputs.errors = Optional.empty();
      inputs.timestamp = Optional.empty();
    }
  }
}
