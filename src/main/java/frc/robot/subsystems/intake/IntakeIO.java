package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double spinMotorSetSpeed = 0.0;
    public double deployMotorSuppliedCurrent = 0.0;
    public double spinMotorSuppliedCurrent = 0.0;
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

  public default void setDeployMotorRequest(ControlRequest request) {}

  public default void setSpinMotorSpeed(double speed) {}

  public default void setDeployMotorSpeed(double speed) {}

  public default void setDeployTargetAngle(Angle angle) {}

  public default Angle getDeployTargetAngle() {
    return Degrees.of(0.0);
  }

  public default void setDeployGains(Slot0Configs gains) {}
}
