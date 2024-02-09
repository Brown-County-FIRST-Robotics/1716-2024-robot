package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

/** Field positions of game components */
public class FieldConstants {
  /**
   * Gets the position of the speaker
   *
   * @return The position of the speaker, based on alliance color
   */
  public static Translation3d getSpeaker() {
    return flip(new Translation3d(0.458597, 5.544566, 2.1105114));
  }

  /**
   * Flips the translation based on alliance
   *
   * @param inp The position for the blue alliance
   * @return The position for the FMS alliance
   */
  public static Translation3d flip(Translation3d inp) {
    return new Translation3d(
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue
            ? inp.getX()
            : 16.541242 - inp.getX(),
        inp.getY(),
        inp.getZ());
  }

  /**
   * Flips the translation based on alliance
   *
   * @param inp The position for the blue alliance
   * @return The position for the FMS alliance
   */
  public static Translation2d flip(Translation2d inp) {
    return flip(new Translation3d(inp.getX(), inp.getY(), 0)).toTranslation2d();
  }
}
