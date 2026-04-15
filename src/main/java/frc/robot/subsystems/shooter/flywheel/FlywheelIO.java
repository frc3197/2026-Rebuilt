package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelInputs {
    public double flywheelTargetVelocity = 0.0;
    public double flywheelVelocityActual = 0.0;
    public MutVoltage flywheelSuppliedVoltage = new MutVoltage(0.0, 0.0, Volts);
    public MutCurrent flywheelCurrentDraw = new MutCurrent(0.0, 0.0, Amps);
  }

  public class FlywheelParameters {
    public AngularVelocity rpsError = RotationsPerSecond.of(0.0);
  }

  public default void setFlywheelTargetVelocity(AngularVelocity velocity) {}

  public default FlywheelParameters getFlywheelParameters() {
    return new FlywheelParameters();
  }

  public default void updateFlywheelSlot0Configs(Slot0Configs newConfig) {}

  public default void setFlywheelOutput(ControlRequest request) {}

  public default void updateInputs(FlywheelInputs inputs) {}

  public default void setActive() {}

  public default void setInactive() {}
}
