package frc.robot.subsystems.intake;

<<<<<<< Updated upstream
import static edu.wpi.first.units.Units.Degrees;

=======
>>>>>>> Stashed changes
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
<<<<<<< Updated upstream
import frc.robot.subsystems.intake.IntakeIO;
=======
>>>>>>> Stashed changes

public class IntakeIOTalonFX implements IntakeIO {

<<<<<<< Updated upstream
    private final TalonFX spinMotor;
    
    public IntakeIOTalonFX() {
        deployMotor    = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID);
        deployCANcoder = new CANcoder(IntakeConstants.DEPLOY_CANCODER_ID);

        spinMotor      = new TalonFX(IntakeConstants.SPIN_MOTOR_ID);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.deployAngle = getDeployMotorAbsPos();
        inputs.spinMotorSetSpeed = getSpinMotorSpeed();

        inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
        inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
    } 

    @Override
    public Angle getDeployMotorAbsPos() {
        return deployCANcoder.getAbsolutePosition().getValue();
    }

    @Override
    public Double getSpinMotorSpeed() {
        return spinMotor.get();
    }

    @Override
    public Double getDeployMotorSpeed() {
        return deployMotor.get();
    }

    @Override
    public void setSpinMotorSpeed(Double speed) {
        spinMotor.set(speed);
    }

    @Override
    public void setDeployMotorSpeed(Double speed) {
        deployMotor.set(speed);
    }
=======
  private final TalonFX deployMotor;
  private final CANcoder deployCANcoder;

  private final TalonFX spinMotor;
  private final CANcoder spinCANcoder;

  public IntakeIOTalonFX() {
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID);
    deployCANcoder = new CANcoder(IntakeConstants.DEPLOY_CANCODER_ID);

    spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID);
    spinCANcoder = new CANcoder(IntakeConstants.SPIN_CANCODER_ID);
  }

  @Override
  public Angle GetSpinMotorAbsPos() {
    return spinCANcoder.getAbsolutePosition().getValue();
  }

  @Override
  public Angle GetDeployMotorAbsPos() {
    return deployCANcoder.getAbsolutePosition().getValue();
  }

  @Override
  public void SetDeployMotorSpeed(Double speed) {
    deployMotor.set(speed);
  }

  @Override
  public void SetSpinMotorSpeed(Double speed) {
    spinMotor.set(speed);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {}
>>>>>>> Stashed changes
}
