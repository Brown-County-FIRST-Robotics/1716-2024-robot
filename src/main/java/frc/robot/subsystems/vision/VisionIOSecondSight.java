package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.*;

public class VisionIOSecondSight implements VisionIO{
  BooleanSubscriber isRecordingSub;
  StringSubscriber recordingPathSub;
  StringArraySubscriber idsSub;
  DoubleArraySubscriber posesSub;

  public VisionIOSecondSight(String name){
    NetworkTable table=NetworkTableInstance.getDefault().getTable(name);
    isRecordingSub=table.getBooleanTopic("isRecording").subscribe(false);
    recordingPathSub=table.getStringTopic("recordingPath").subscribe("");
    idsSub=table.getStringArrayTopic("ids").subscribe(new String[]{});
    posesSub=table.getDoubleArrayTopic("poses").subscribe(new double[]{});
  }
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isRecording=isRecordingSub.get();
    inputs.recordingPath= recordingPathSub.get();
    TimestampedStringArray[] ids=idsSub.readQueue();
    double[][] poses=posesSub.readQueueValues();
    inputs.timestamps=new double[ids.length];
    inputs.ids=new String[ids.length][];
    inputs.poses=poses.clone();
    for (int i = 0; i < ids.length; i++) {
      inputs.timestamps[i]=ids[i].timestamp/1000000.0;
      inputs.ids[i]=ids[i].value;
    }
  }
}
