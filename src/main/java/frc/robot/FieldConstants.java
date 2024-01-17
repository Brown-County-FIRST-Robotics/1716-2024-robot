package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
  public static Translation3d getSpeaker() {
    return flip(new Translation3d(0.458597, 5.544566, 2.1105114));
  }

  public static Translation3d flip(Translation3d inp) {
    return new Translation3d(
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue
            ? inp.getX()
            : 16.541242 - inp.getX(),
        inp.getY(),
        inp.getZ());
  }

  public static Translation2d flip(Translation2d inp) {
    return flip(new Translation3d(inp.getX(), inp.getY(), 0)).toTranslation2d();
  }
}
