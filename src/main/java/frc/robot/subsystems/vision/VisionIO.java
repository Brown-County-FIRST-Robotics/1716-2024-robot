package frc.robot.subsystems.vision;

import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** The IO layer for one camera */
public interface VisionIO {
  /** The inputs from a camera */
  public static class VisionIOInputs implements LoggableInputs {
    /** The pose from the camera */
    public Optional<double[]> pose = Optional.empty();
    /** The timestamp of the frame */
    public Optional<Double> timestamp = Optional.empty();
    /** The IDs of the apriltags detected */
    public Optional<String[]> ids = Optional.empty();
    /** The RMS error of the detection */
    public Optional<Double> errors = Optional.empty();
    /** If the camera is recording */
    public boolean isRecording = false;
    /** The current path of the video file */
    public String recordingPath = "";

    @Override
    public void toLog(LogTable table) {
      table.put("RecordingPath", recordingPath);
      table.put("IsRecording", isRecording);
      table.put("hasPose", pose.isPresent());
      table.put("hasError", errors.isPresent());
      table.put("hasIDs", ids.isPresent());
      table.put("hasTimestamp", timestamp.isPresent());
      table.put("Pose", pose.orElse(new double[] {}));
      table.put("Error", errors.orElse(0.0));
      table.put("IDs", ids.orElse(new String[] {}));
      table.put("Timestamp", timestamp.orElse(0.0));
    }

    @Override
    public void fromLog(LogTable table) {
      pose =
          table.get("hasPose", false)
              ? Optional.of(table.get("Pose", new double[] {}))
              : Optional.empty();
      timestamp =
          table.get("hasTimestamp", false)
              ? Optional.of(table.get("Timestamp", 0.0))
              : Optional.empty();
      ids =
          table.get("hasIDs", false)
              ? Optional.of(table.get("IDs", new String[] {}))
              : Optional.empty();
      errors =
          table.get("hasError", false) ? Optional.of(table.get("Error", 0.0)) : Optional.empty();
      recordingPath = table.get("RecordingPath", "");
      isRecording = table.get("IsRecording", false);
    }
  }

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  default void updateInputs(VisionIOInputs inputs) {}
}
