package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Overrides;
import frc.robot.utils.PeriodicRunnable;
import frc.robot.utils.ShootWhileMove;
import org.littletonrobotics.junction.Logger;

/** The vision subsystem */
public class Vision extends PeriodicRunnable {
  Transform3d[] camPoses;
  VisionIO[] ios;
  VisionIOInputs[] inputs;
  Drivetrain drivetrain;
  AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  LoggedTunableNumber oneTagTranslationStdDev =
      new LoggedTunableNumber("Vision/One tag Translation StdDev", 1);
  LoggedTunableNumber oneTagRotationStdDev =
      new LoggedTunableNumber("Vision/One tag Rotation StdDev", 1);
  LoggedTunableNumber multiTagTranslationStdDev =
      new LoggedTunableNumber("Vision/Multi tag Translation StdDev", 0.05);
  LoggedTunableNumber multiTagRotationStdDev =
      new LoggedTunableNumber("Vision/Multi tag Rotation StdDev", 1);
  LoggedTunableNumber maxRMSError = new LoggedTunableNumber("Vision/Max RMS Error", 0.85);

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
      if (inputs[i].pose.isPresent() && inputs[i].timestamp.isPresent()) {
        Pose3d outPose = inputs[i].pose.get();
        Pose3d poseOfBot = outPose;
        Logger.recordOutput("Vision/EstPose_" + i, poseOfBot);
        if (!Overrides.disableVision.get()
            && (ShootWhileMove.getFieldRelativeSpeeds(drivetrain.getVelocity(), new Rotation2d())
                    .getNorm()
                < 0.8)
            && Math.abs(drivetrain.getVelocity().omegaRadiansPerSecond) < 0.5) {
          drivetrain.addVisionUpdate(
              poseOfBot.toPose2d(), VecBuilder.fill(0.1, 0.1, 1), inputs[i].timestamp.get());
        }
      }
    }
  }
}
