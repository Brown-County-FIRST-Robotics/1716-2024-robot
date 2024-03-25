package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    // TEMP: fudge factors
    return flip(new Translation3d(-.04 + 5 * .0254, 5.55, 80 * 0.0254));
  }

  public static Translation2d getGamePiece(int ind) {
    /* Game Pieces (blue alliance):
     * _______________________________________________
     * |     |_|
     * |                                           (7)
     * |---|         (2)
     * |   |         (1)                           (6)
     * |---|         (0)
     * |                                           (5)
     * |
     * |                                           (4)
     * |
     * |                                           (3)
     * |______________________________________________
     * */
    Translation2d center = new Translation2d(16.541242 / 2, (29.64 + 66.0 + 66.0) * 0.0254);
    return switch (ind) {
      case 0 -> flip(new Translation2d(114 * 0.0254, center.getY()));
      case 1 -> getGamePiece(0).plus(new Translation2d(0, 0.0254 * 57));
      case 2 -> getGamePiece(1).plus(new Translation2d(0, 0.0254 * 57));
      case 3 -> getGamePiece(4).minus(new Translation2d(0, 0.0254 * 66));
      case 4 -> center.minus(new Translation2d(0, 0.0254 * 66));
      case 5 -> center;
      case 6 -> center.plus(new Translation2d(0, 0.0254 * 66));
      case 7 -> getGamePiece(6).plus(new Translation2d(0, 0.0254 * 66));
      default -> throw new IllegalArgumentException(
          "ind parameter must be 0-7 (inclusive) ind=" + ind);
    };
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
   * Flips the rotation based on alliance
   *
   * @param inp The rotation when on the blue alliance
   * @return The rotation for the FMS alliance
   */
  public static Rotation2d flip(Rotation2d inp) {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue
        ? inp
        : new Rotation2d(-inp.getCos(), inp.getSin());
  }
  /**
   * Flips the pose based on alliance
   *
   * @param inp The pose when on the blue alliance
   * @return The pose for the FMS alliance
   */
  public static Pose2d flip(Pose2d inp) {
    return new Pose2d(flip(inp.getTranslation()), flip(inp.getRotation()));
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
