package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.HardwareID;

public class ShooterConstants implements HardwareID.ShooterHardwareID {

  public static final PIDController FLYWHEEL_PID_CONTROLLER = new PIDController(0.0, 0.0, 0.0);

  // Describes the turret's bottom opening relative to robot position
  public static final Pose3d ROBOT_TO_TURRET_CENTER =
      new Pose3d(new Translation3d(0.3, 1.0, 0.5), Rotation3d.kZero);
}
