package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelInputs {
    public MutAngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0).mutableCopy();
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

  public default void setFlywheelOutput(ControlRequest request) {}

  public default void updateInputs(FlywheelInputs inputs) {}
}
