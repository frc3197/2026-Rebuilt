package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID;

public class ShooterConstants implements HardwareID.ShooterHardwareID {

  public static final PIDController FLYWHEEL_PID_CONTROLLER = new PIDController(0.0, 0.0, 0.0);

  public static final Voltage MAX_TURRET_ROTATION_MOTOR_VOLTS = Volts.of(10.0);

  // Describes the turret's bottom opening relative to robot position
  public static final Transform3d ROBOT_TO_TURRET_CENTER =
      new Transform3d(0.2, 0.2, 0.5, Rotation3d.kZero);

  public static final PIDController TURRET_ANGLE_P_PID_CONTROLLER =
      new PIDController(0.5, 0.0, 0.0);
}
