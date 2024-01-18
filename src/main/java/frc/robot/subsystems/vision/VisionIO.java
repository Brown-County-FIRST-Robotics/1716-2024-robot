package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** The IO layer for one camera */
public interface VisionIO {
  /** The inputs from a camera */
  public static class VisionIOInputs implements LoggableInputs {
    /** The pose from the camera */
    public double[][] poses = {};
    /** The timestamp of the frame */
    public double[] timestamps = {};
    /** The IDs of the apriltags detected */
    public String[][] ids = {};
    /** The RMS error of the detection */
    public double[] errors = {};
    /** If the camera is recording */
    public boolean isRecording = false;
    /** The current path of the video file */
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

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  default void updateInputs(VisionIOInputs inputs) {}
}
