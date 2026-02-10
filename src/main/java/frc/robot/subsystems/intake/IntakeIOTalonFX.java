package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.HardwareID;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX deployMotor;
  private final TalonFX spinMotor;

  private final CANcoder deployEncoder;

  public IntakeIOTalonFX() {
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, HardwareID.MAIN_CANBUS);

    spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID, HardwareID.MAIN_CANBUS);

    deployEncoder = new CANcoder(IntakeConstants.DEPLOY_CANCODER_ID, HardwareID.MAIN_CANBUS);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.spinMotorSetSpeed = getSpinMotorSpeed();

    inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
    inputs.spinMotorSuppliedCurrent = spinMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void configureMotors() {
    //deployMotor
    //    .getConfigurator()
    //    .apply(IntakeConstants.motorConfigurationConstants.spinMotorConfig);
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
  public Supplier<Angle> getDeployMotorAbsPos() {
    return deployEncoder.getAbsolutePosition().asSupplier();
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
