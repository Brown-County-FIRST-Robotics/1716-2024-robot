package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

/** The IO layer for one SecondSight camera */
public class VisionIOSecondSight implements VisionIO {
  BooleanSubscriber isRecordingSub;
  StringSubscriber recordingPathSub;
  StringArraySubscriber idsSub;
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
                new String[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    posesSub =
        table
            .getDoubleArrayTopic("Pose")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    errorSub =
        table
            .getDoubleTopic("RMSError")
            .subscribe(-1, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    idsSub.readQueue();
    posesSub.readQueue();
    errorSub.readQueue();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isRecording = isRecordingSub.get();
    inputs.recordingPath = recordingPathSub.get();
    var unused=posesSub.readQueue();
    System.out.println(Arrays.toString(unused));
    double[][] poses = posesSub.readQueueValues();
    String[][] ids = idsSub.readQueueValues();
    inputs.timestamps = new double[ids.length];
    inputs.ids = new String[ids.length][];
    inputs.poses = poses.clone();
    for (int i = 0; i < ids.length; i++) {
      inputs.timestamps[i] = Timer.getFPGATimestamp();
      inputs.ids[i] = ids[i];
    }
    inputs.errors = errorSub.readQueueValues();
  }
}
