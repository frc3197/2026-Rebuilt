package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.HardwareID;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX deployMotor;
  private final TalonFX spinMotor;

  public IntakeIOTalonFX() {
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, HardwareID.MAIN_CANBUS);

    spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID, HardwareID.MAIN_CANBUS);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.spinMotorSetSpeed = getSpinMotorSpeed();

    inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
    inputs.spinMotorSuppliedCurrent = spinMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void configureMotors() {
    deployMotor
        .getConfigurator()
        .apply(IntakeConstants.motorConfigurationConstants.spinMotorConfig);
  }

  @Override
  public double getSpinMotorSpeed() {
    return spinMotor.get();
  }

  @Override
  public double getDeployMotorSpeed() {
    return deployMotor.get();
  }

  @Override
  public void setSpinMotorSpeed(double speed) {
    spinMotor.set(speed);
  }

  @Override
  public void setDeployMotorSpeed(double speed) {
    deployMotor.set(speed);
  }
}
