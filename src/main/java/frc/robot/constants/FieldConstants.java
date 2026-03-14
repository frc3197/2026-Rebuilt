package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  public static final Distance HUB_HEIGHT = Inches.of(67);

  public static final Pose2d CLIMB_ALIGN_POSE_LEFT = new Pose2d(1.9, 4.145, Rotation2d.kCCW_90deg);
  public static final Pose2d CLIMB_ALIGN_POSE_RIGHT = new Pose2d(1.9, 3.297, Rotation2d.kCCW_90deg);

  public static final Pose2d PASSING_UPPER = new Pose2d(2.163, 5.720, Rotation2d.kZero);
  public static final Pose2d PASSING_LOWER = new Pose2d(2.163, 1.821, Rotation2d.kZero);

  public class Blue {
    public static final Pose2d HUB_CENTER = new Pose2d(4.593, 4.026, new Rotation2d(0.0));

    public static final double MAX_HUB_TRACKING_X = 5.087;

    public static final Rectangle2d BOTTOM_TRENCH_RECTANGLE =
        new Rectangle2d(new Pose2d(4.600, 0.0, Rotation2d.kZero), 1.2, 2.6);

    public static final Rectangle2d TOP_TRENCH_RECTANGLE =
        new Rectangle2d(new Pose2d(4.600, 8.100, Rotation2d.kZero), 1.2, 2.6);
  }

  public class Red {
    public static final Pose2d HUB_CENTER = new Pose2d(11.906, 4.026, new Rotation2d(0.0));

    public static final double MIN_HUB_TRACKING_X = 11.453;

    public static final Rectangle2d BOTTOM_TRENCH_RECTANGLE =
        new Rectangle2d(new Pose2d(11.9, 0.0, Rotation2d.kZero), 1.2, 2.6);

    public static final Rectangle2d TOP_TRENCH_RECTANGLE =
        new Rectangle2d(new Pose2d(11.9, 8.100, Rotation2d.kZero), 1.2, 2.6);
  }
}
