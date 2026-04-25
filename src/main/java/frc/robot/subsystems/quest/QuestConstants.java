package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

public class QuestConstants {

  private static final Distance ROBOT_TO_QUEST_X = Inches.of(-8.6);
  private static final Distance ROBOT_TO_QUEST_Y = Inches.of(-8.3);
  private static final Distance ROBOT_TO_QUEST_Z = Inches.of(12.125);
  private static final Rotation3d ROBOT_TO_QUEST_ROTATION =
      new Rotation3d(0, 0, Degrees.of(-45 + 180).in(Radians));

  public static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          ROBOT_TO_QUEST_Y.in(Meters),
          -ROBOT_TO_QUEST_X.in(Meters),
          ROBOT_TO_QUEST_Z.in(Meters),
          ROBOT_TO_QUEST_ROTATION);
}
