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
                new String[] {"asd"}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true),PubSubOption.pollStorage(3));
    posesSub =
        table
            .getDoubleArrayTopic("Pose")
            .subscribe(
                new double[] {0,0,0,0,0,0,0}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true),PubSubOption.pollStorage(3));
    errorSub =
        table
            .getDoubleTopic("RMSError")
            .subscribe(-1.0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true),PubSubOption.pollStorage(3));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    TimestampedStringArray[] ids=idsSub.readQueue();
    inputs.isRecording = isRecordingSub.get();
    inputs.recordingPath = recordingPathSub.get();
    var t=idsSub.getLastChange();
    TimestampedDoubleArray[] poses=posesSub.readQueue();
    TimestampedDouble[] errors=errorSub.readQueue();
    if(ids.length!=poses.length){
      ids=new TimestampedStringArray[]{};
      poses=new TimestampedDoubleArray[]{};
    }
    inputs.poses=new double[poses.length][];
    inputs.ids=new String[poses.length][];
    inputs.timestamps=new double[poses.length];
    inputs.errors=new double[poses.length];
    for (int i = 0; i < ids.length; i++) {
      inputs.timestamps[i]=poses[i].timestamp/1000000.0;
      inputs.ids[i]=ids[i].value;
      inputs.poses[i]=poses[i].value;
      if(ids.length==errors.length){
        inputs.errors[i]=errors[i].value;
      }
    }
  }
}
