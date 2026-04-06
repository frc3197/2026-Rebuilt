package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double spinMotorSetSpeed = 0.0;
    public MutCurrent deployMotorSuppliedCurrent = Amps.of(0.0).mutableCopy();
    public MutCurrent leftSpinMotorSuppliedCurrent = Amps.of(0.0).mutableCopy();
    public MutCurrent rightSpinMotorSuppliedCurrent = Amps.of(0.0).mutableCopy();
    public double deployAngleDegrees = 0.0;
    public MutAngularVelocity deployMotorVelocity = RotationsPerSecond.of(0.0).mutableCopy();
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void configureMotors() {}

  public default double getSpinMotorSpeed() {
    return 0.0;
  }

  public default double getDeployMotorSpeed() {
    return 0.0;
  }

  public default Angle getDeployMotorAbsPos() {
    return Degrees.of(0.0);
  }

  public default void setSpinMotorSpeed(double speed) {}

  public default void setSpinMotorRequest(ControlRequest request) {}

  public default void setDeployMotorRequest(ControlRequest request) {}

  public default void setDeployMotorSpeed(double speed) {}

  public default void setDeployTargetAngle(Angle angle) {}

  public default Angle getDeployTargetAngle() {
    return Degrees.of(0.0);
  }

  public default void setDeployGains(Slot0Configs gains) {}
}
