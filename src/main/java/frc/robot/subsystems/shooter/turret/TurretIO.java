package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretInputs {
    public MutAngle turretMotorAngle = Degrees.of(0.0).mutableCopy();
    public MutAngle hoodAngle = Degrees.of(45.0).mutableCopy();

    public MutVoltage turretAngleSuppliedVoltage = new MutVoltage(0.0, 0.0, Volts);
    public MutCurrent turretAngleCurrentDraw = new MutCurrent(0.0, 0.0, Amps);
  }

  public class TurretParameters {
    public Angle turretRotationError = Degrees.of(0.0);
    public Angle turretRotation = Degrees.of(0.0);
    public Angle turretRotationTarget = Degrees.of(0.0);
  }

  public default void setTurretMotorVolts(Voltage volts) {}

  public default void setTurretControlRequest(ControlRequest reuest) {}

  public default void zeroTurretEncoder() {}

  public default void updateTurretSlot0Configs(Slot0Configs newConfig) {}

  public default void setOutputTargetAngle(Angle rotations) {}
  ;

  public default TurretParameters getTurretParameters() {
    return new TurretParameters();
  }

  public default void updateInputs(TurretInputs inputs) {}
}
