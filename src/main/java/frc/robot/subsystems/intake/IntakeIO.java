package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface IntakeIO {

    @AutoLog
    public static class IntakeInputs {
        public Double spinMotorSetSpeed = 0.0;

        public Double deployMotorSuppliedCurrent = 0.0;
        public Double spinMotorSuppliedCurrent = 0.0;
    }

      public default void updateInputs(IntakeInputs inputs) {};

      public default Double getSpinMotorSpeed() { return null; };
      public default Double getDeployMotorSpeed() { return null; };

      public default void setSpinMotorSpeed(Double speed) {};
      public default void setDeployMotorSpeed(Double speed) {};

}
