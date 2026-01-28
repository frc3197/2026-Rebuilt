package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

<<<<<<< Updated upstream
<<<<<<< Updated upstream
    @AutoLog
    public static class IntakeInputs {
        public Double spinMotorSetSpeed = 0.0;
=======
=======
>>>>>>> Stashed changes
  @AutoLog
  public static class IntakeInputs {
    public Angle deployAngle = Degrees.of(0);
    public double spinMotorSetSpeed = 0.0;
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

    public double deployMotorSuppliedCurrent = 0.0;
    public double spinMotorSuppliedCurrent = 0.0;
  }

<<<<<<< Updated upstream
<<<<<<< Updated upstream
      public default void updateInputs(IntakeInputs inputs) {};

      public default Double getSpinMotorSpeed() { return null; };
      public default Double getDeployMotorSpeed() { return null; };

      public default void setSpinMotorSpeed(Double speed) {};
      public default void setDeployMotorSpeed(Double speed) {};
=======
  public default void updateInputs(IntakeInputs inputs) {}
  ;

  public default Angle GetSpinMotorAbsPos() {
=======
  public default void updateInputs(IntakeInputs inputs) {}
  ;

  public default Angle GetSpinMotorAbsPos() {
    return null;
  }
  ;

  public default Angle GetDeployMotorAbsPos() {
>>>>>>> Stashed changes
    return null;
  }
  ;

<<<<<<< Updated upstream
  public default Angle GetDeployMotorAbsPos() {
    return null;
  }
  ;

  public default void SetDeployMotorSpeed(Double speed) {}
  ;
>>>>>>> Stashed changes

=======
  public default void SetDeployMotorSpeed(Double speed) {}
  ;

>>>>>>> Stashed changes
  public default void SetSpinMotorSpeed(Double speed) {}
  ;
}
