package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {

  Double spinMotorSpeed = 0.0;
  Double deployMotorSpeed = 0.0;

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.spinMotorSetSpeed = getSpinMotorSpeed();

    // inputs.deployMotorSuppliedCurrent =
    // deployMotor.getSupplyCurrent().getValueAsDouble();
    // inputs.spinMotorSuppliedCurrent =
    // spinMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void configureMotors() {}

  @Override
  public double getSpinMotorSpeed() {
    return spinMotorSpeed;
  }

  @Override
  public double getDeployMotorSpeed() {
    return deployMotorSpeed;
  }

  @Override
  public void setSpinMotorSpeed(double speed) {
    spinMotorSpeed = speed;
  }

  @Override
  public void setDeployMotorSpeed(double speed) {
    deployMotorSpeed = speed;
  }
}
