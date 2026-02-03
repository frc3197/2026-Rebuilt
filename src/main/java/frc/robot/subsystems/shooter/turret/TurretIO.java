package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretInputs {
    public Angle turretAngle = Degrees.of(0.0);
    public Angle hoodAngle = Degrees.of(45.0);

    public MutVoltage turretAngleSuppliedVoltage = new MutVoltage(0.0, 0.0, Volts);
    public MutCurrent turretAngleCurrentDraw = new MutCurrent(0.0, 0.0, Amps);
  }

  public class TurretParameters {
    public Angle turretRotationError = Degrees.of(0.0);
  }

  public default void setTurretMotorVolts(Voltage volts) {}

  public default TurretParameters getTurretParameters() {
    return new TurretParameters();
  }

  public default void updateInputs(TurretInputs inputs) {}
}
