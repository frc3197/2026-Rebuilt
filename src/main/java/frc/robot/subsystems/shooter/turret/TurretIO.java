package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretInputs {
    public Angle turretAngle = Degrees.of(0.0);
    public Angle hoodAngle = Degrees.of(45.0);
    public AngularVelocity flywheelRPS = RotationsPerSecond.of(0.0);

    public double flywheelSuppliedVoltage = 0.0;
    public double hoodSuppliedVoltage = 0.0;
    public double turretAngleSuppliedVoltage = 0.0;
  }

  public class TurretParameters {
    public Angle turretRotationError = Degrees.of(0.0);

    public TurretParameters() {}
  }

  public default void setTurretMotorVolts(Voltage volts) {}
  ;

  public default TurretParameters getTurretParameters() {
    return new TurretParameters();
  }
  ;

  public default void updateInputs(TurretInputs inputs) {}
  ;
}
