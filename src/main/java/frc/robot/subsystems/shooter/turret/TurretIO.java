package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretInputs {
    public MutAngle turretMotorAngle = Degrees.of(0.0).mutableCopy();
    public MutCurrent turretMotorCurrent = Amps.of(0.0).mutableCopy();
    public MutVoltage turretMotorVoltage = Volts.of(0.0).mutableCopy();
    public MutAngle hoodAngle = Degrees.of(45.0).mutableCopy();
    public MutDistance hoodActuatorExtension = Millimeters.of(0.0).mutableCopy();
    public MutDistance hoodActuatorExtensionTarget = Millimeters.of(0.0).mutableCopy();
    public boolean limitSwitchActivated = false;
  }

  public class TurretParameters {
    public MutAngle turretRotationError = Degrees.of(0.0).mutableCopy();
    public MutAngle turretRotation = Degrees.of(0.0).mutableCopy();
    public MutAngle turretRotationTarget = Degrees.of(0.0).mutableCopy();
    public MutDistance hoodActuatorExtension = Millimeters.of(0.0).mutableCopy();
    public MutDistance hoodActuatorExtensionTarget = Millimeters.of(0.0).mutableCopy();
    public MutAngle hoodAngle = Degrees.of(0.0).mutableCopy();
  }

  public default void setTurretMotorVolts(Voltage volts) {}

  public default void setTurretControlRequest(ControlRequest reuest) {}

  public default void zeroTurretEncoder() {}

  public default void updateTurretSlot0Configs(Slot0Configs newConfig) {}

  public default void updateTurretMMConfigs(MotionMagicConfigs newConfig) {}

  public default void setOutputTargetAngle(Angle rotations) {}

  public default TurretParameters getTurretParameters() {
    return new TurretParameters();
  }

  public default void setHoodActuatorMM(Distance d) {}

  public default void updateInputs(TurretInputs inputs) {}

  public default boolean limitActivated() {
    return true;
  }
}
