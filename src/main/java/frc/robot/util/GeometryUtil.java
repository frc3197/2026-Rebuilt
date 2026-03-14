package frc.robot.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

public class GeometryUtil {

  // Sample 5 instances of points along the velocity vector (field-centric)
  public static boolean intersects(
      Rectangle2d rectangle, Translation2d robotPosition, Vector<N2> velocityVector) {
    for (int i = 0; i < 5; i++) {
      if (rectangle.contains(
          robotPosition.plus(
              new Translation2d(
                  (double) i / 5.0 * velocityVector.get(0),
                  (double) i / 5.0 * velocityVector.get(1))))) return true;
    }
    return false;
  }
}
