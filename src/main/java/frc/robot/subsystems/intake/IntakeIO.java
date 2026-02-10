package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double spinMotorSetSpeed = 0.0;

    public double deployMotorSuppliedCurrent = 0.0;
    public double spinMotorSuppliedCurrent = 0.0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void configureMotors() {}

  public default double getSpinMotorSpeed() {
    return 0.0;
  }

  public default double getDeployMotorSpeed() {
    return 0.0;
  }

  public default Supplier<Angle> getDeployMotorAbsPos() {
    return null;
  }

  public default void setSpinMotorSpeed(double speed) {}

  public default void setDeployMotorSpeed(double speed) {}
}
