package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface IntakeIO {

    @AutoLog
    public static class IntakeInputs {
        public Angle deployAngle = Degrees.of(0);
        public boolean isDeployed = false;

        public Double spinMotorSetSpeed = 0.0;

        public Double deployMotorSuppliedCurrent = 0.0;
        public Double spinMotorSuppliedCurrent = 0.0;
    }

      public default void updateInputs(IntakeInputs inputs) {};
      public default Angle GetSpinMotorAbsPos() { return null; };

      public default Angle GetDeployMotorAbsPos() { return null; };

      public default void SetDeployMotorSpeed(Double speed) {};

      public default void SetSpinMotorSpeed(Double speed) {};


}
