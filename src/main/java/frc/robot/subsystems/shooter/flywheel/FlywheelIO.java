package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelInputs {
    public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0);

    public MutVoltage flywheelSuppliedVoltage = new MutVoltage(0.0, 0.0, Volts);
    public MutCurrent flywheelCurrentDraw = new MutCurrent(0.0, 0.0, Amps);
  }

  public class FlywheelParameters {
    public AngularVelocity rpsError = RotationsPerSecond.of(0.0);
  }

  public default void setFlywheelMotorVolts(Voltage volts) {}

  public default void setFlywheelTargetVelocity(AngularVelocity velocity) {}

  public default FlywheelParameters getFlywheelParameters() {
    return new FlywheelParameters();
  }

  public default void updateInputs(FlywheelInputs inputs) {}
}
