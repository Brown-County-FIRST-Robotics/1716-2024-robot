package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** The IO interface for one camera */
public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public double[][] poses = {};
    public double[] timestamps = {};
    public String[][] ids = {};
    public double[] errors = {};
    public boolean isRecording = false;
    public String recordingPath = "";

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("RecordingPath", recordingPath);
      table.put("IsRecording", isRecording);
      table.put("Errors", errors);
      table.put("NumLoops", ids.length);
      assert ids.length == poses.length;
      for (int i = 0; i < ids.length; i++) {
        table.put("IDs/" + i, ids[i]);
        table.put("Poses/" + i, poses[i]);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", new double[] {});
      recordingPath = table.get("RecordingPath", "");
      isRecording = table.get("IsRecording", false);
      errors = table.get("Errors", new double[] {});
      int nmLoops = (int) table.get("NumLoops", 0);
      ids = new String[nmLoops][];
      poses = new double[nmLoops][];
      for (int i = 0; i < nmLoops; i++) {
        ids[i] = table.get("IDs/" + i, new String[] {});
        poses[i] = table.get("Poses/" + i, new double[] {});
      }
    }
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
