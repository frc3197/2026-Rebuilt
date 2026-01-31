package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX deployMotor;

  private final TalonFX spinMotor;

  public IntakeIOTalonFX() {
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID);

    spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.spinMotorSetSpeed = getSpinMotorSpeed();

    inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
    inputs.spinMotorSuppliedCurrent = spinMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void configureMotors() {
    deployMotor.getConfigurator().apply(IntakeConstants.motorConfigurationConstants.spinMotorConfig);
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
}
