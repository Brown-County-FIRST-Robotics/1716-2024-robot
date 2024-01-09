package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;

public class VisionIOSecondSight implements VisionIO {
  BooleanSubscriber isRecordingSub;
  StringSubscriber recordingPathSub;
  StringArraySubscriber idsSub;
  DoubleArraySubscriber posesSub;
  DoubleSubscriber errorSub;

  public VisionIOSecondSight(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SS_LAPTOP").getSubTable("0");
    isRecordingSub = table.getBooleanTopic("isRecording").subscribe(false);
    recordingPathSub = table.getStringTopic("recordingPath").subscribe("");
    idsSub = table.getStringArrayTopic("IDs").subscribe(new String[] {},PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    posesSub = table.getDoubleArrayTopic("Pose").subscribe(new double[] {},PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    errorSub = table.getDoubleTopic("RMSError").subscribe(-1,PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isRecording = isRecordingSub.get();
    inputs.recordingPath = recordingPathSub.get();
    TimestampedStringArray[] ids = idsSub.readQueue();
    double[][] poses = posesSub.readQueueValues();
    inputs.timestamps = new double[ids.length];
    inputs.ids = new String[ids.length][];
    inputs.poses = poses.clone();
    for (int i = 0; i < ids.length; i++) {
      inputs.timestamps[i] = ids[i].timestamp / 1000000.0;
      inputs.ids[i] = ids[i].value;
    }
    inputs.errors = errorSub.readQueueValues();
  }
}
