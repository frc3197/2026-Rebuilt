package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.HardwareID;
import frc.robot.util.RealSubsystem;

public class IntakeIOTalonFX extends RealSubsystem implements IntakeIO {

  private final TalonFX deployMotor;
  private final TalonFX spinMotor;

  private MutAngle targetDeployAngle = IntakeConstants.FULLY_RETRACTED_ANGLE.mutableCopy();

  private final CANcoder deployEncoder;

  public IntakeIOTalonFX() {
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, HardwareID.MAIN_CANBUS);
    spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID, HardwareID.MAIN_CANBUS);
    deployEncoder = new CANcoder(IntakeConstants.DEPLOY_CANCODER_ID, HardwareID.MAIN_CANBUS);

    configureHardware();
  }

  @Override
  protected void configureHardware() {
    deployMotor
        .getConfigurator()
        .apply(
            IntakeConstants.DEPLOY_MOTOR_CONFIG.withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(deployEncoder.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withSensorToMechanismRatio(1.0)
                    .withRotorToSensorRatio(1)));

    spinMotor.getConfigurator().apply(IntakeConstants.SPIN_MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.spinMotorSetSpeed = getSpinMotorSpeed();
    inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
    inputs.spinMotorSuppliedCurrent = spinMotor.getSupplyCurrent().getValueAsDouble();
    inputs.deployAngleDegrees = deployMotor.getPosition().getValue().in(Degrees);
    inputs.deployMotorVelocity.mut_replace(deployEncoder.getVelocity().getValue());
  }

  @Override
  public double getSpinMotorSpeed() {
    return spinMotor.get();
  }

  @Override
  public void setDeployMotorRequest(ControlRequest request) {
    deployMotor.setControl(request);
  }

  @Override
  public void setDeployTargetAngle(Angle angle) {
    targetDeployAngle.mut_replace(angle);
  }

  @Override
  public Angle getDeployTargetAngle() {
    return targetDeployAngle;
  }

  @Override
  public double getDeployMotorSpeed() {
    return deployMotor.get();
  }

  @Override
  public Angle getDeployMotorAbsPos() {
    return deployEncoder.getAbsolutePosition().getValue();
  }

  @Override
  public void setSpinMotorSpeed(double speed) {
    spinMotor.set(speed);
  }

  @Override
  public void setDeployMotorSpeed(double speed) {
    deployMotor.set(speed);
  }

  @Override
  public void setDeployGains(Slot0Configs gains) {
    deployMotor.getConfigurator().apply(gains);
  }
}
