package frc.robot.subsystems.quest;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

public class QuestConstants {

  private static final Distance QUEST_TO_TURRET_X = Inches.of(9.0);
  private static final Distance QUEST_TO_TURRET_Y = Inches.of(8);
  private static final Distance QUEST_TO_TURRET_Z = Inches.of(12);
  private static final Rotation3d ROBOT_TO_QUEST_ROTATION =
      new Rotation3d(0, 0, Degrees.of(45).in(Radians));

  public static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          QUEST_TO_TURRET_Y.in(Meters),
          -QUEST_TO_TURRET_X.in(Meters),
          QUEST_TO_TURRET_Z.in(Meters),
          ROBOT_TO_QUEST_ROTATION);
}
