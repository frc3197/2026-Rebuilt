package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.HardwareID;
import frc.robot.util.RealSubsystem;

public class IntakeIOTalonFX extends RealSubsystem implements IntakeIO {

  private final TalonFX deployMotor;
  private final TalonFX leftSpinMotor;
  private final TalonFX rightSpinMotor;

  private MutAngle targetDeployAngle = IntakeConstants.FULLY_RETRACTED_ANGLE.mutableCopy();

  private final CANcoder deployEncoder;

  public IntakeIOTalonFX() {
    deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, HardwareID.MAIN_CANBUS);
    leftSpinMotor = new TalonFX(IntakeConstants.LEFT_SPIN_MOTOR_ID, HardwareID.MAIN_CANBUS);
    rightSpinMotor = new TalonFX(IntakeConstants.RIGHT_SPIN_MOTOR_ID, HardwareID.MAIN_CANBUS);
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

    leftSpinMotor.getConfigurator().apply(IntakeConstants.SPIN_MOTOR_CONFIG);
    rightSpinMotor.getConfigurator().apply(IntakeConstants.SPIN_MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.spinMotorSetSpeed = getSpinMotorSpeed();
    inputs.deployMotorSuppliedCurrent.mut_replace(deployMotor.getSupplyCurrent().getValue());

    inputs.leftSpinMotorSuppliedCurrent.mut_replace(leftSpinMotor.getSupplyCurrent().getValue());
    inputs.rightSpinMotorSuppliedCurrent.mut_replace(rightSpinMotor.getSupplyCurrent().getValue());

    inputs.deployAngleDegrees = deployMotor.getPosition().getValue().in(Degrees);
    inputs.deployMotorVelocity.mut_replace(deployEncoder.getVelocity().getValue());
  }

  @Override
  public double getSpinMotorSpeed() {
    return rightSpinMotor.get();
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
  public void setDeployMotorSpeed(double speed) {
    deployMotor.set(speed);
  }

  @Override
  public void setDeployGains(Slot0Configs gains) {
    deployMotor.getConfigurator().apply(gains);
  }

  @Override
  public void setSpinMotorSpeed(double speed) {
    leftSpinMotor.set(-speed);
    rightSpinMotor.set(speed);
  }

  @Override
  public void setSpinMotorRequest(ControlRequest request) {
    rightSpinMotor.setControl(request);
    leftSpinMotor.setControl(
        new Follower(IntakeConstants.RIGHT_SPIN_MOTOR_ID, MotorAlignmentValue.Opposed));
  }
}
