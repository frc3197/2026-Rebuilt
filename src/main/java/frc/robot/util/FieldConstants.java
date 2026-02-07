package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
  public class Blue {
    public static final Pose2d HUB_CENTER = new Pose2d(4.593, 4.026, new Rotation2d(0.0));

    public static final double MAX_HUB_TRACKING_X = 5.087;
  }

  public class Red {
    public static final Pose2d HUB_CENTER = new Pose2d(11.906, 4.026, new Rotation2d(0.0));

    public static final double MIN_HUB_TRACKING_X = 11.453;
  }
}
