package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterInputs {
    public Angle turretAngle = Degrees.of(0.0);
    public Angle hoodAngle = Degrees.of(45.0);

    public double flywheelSuppliedVoltage = 0.0;
    public double turretSuppliedVoltage = 0.0;
  }

  public default void updateInputs(ShooterInputs inputs) {}
  ;
}
