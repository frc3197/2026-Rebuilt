package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public Angle deployAngle = Degrees.of(0);
    public double spinMotorSetSpeed = 0.0;

    public double deployMotorSuppliedCurrent = 0.0;
    public double spinMotorSuppliedCurrent = 0.0;
  }

  public default void updateInputs(IntakeInputs inputs) {}
  ;

  public default Double getSpinMotorSpeed() {
    return null;
  }
  ;

  public default Double getDeployMotorSpeed() {
    return null;
  }
  ;

  public default void setSpinMotorSpeed(Double speed) {}
  ;

  public default void setDeployMotorSpeed(Double speed) {}
  ;
}
