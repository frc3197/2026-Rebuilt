package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intake.IntakeIO;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX deployMotor;
    private final CANcoder deployCANcoder;

    private final TalonFX spinMotor;

    public IntakeIOTalonFX() {
        deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID);
        deployCANcoder = new CANcoder(IntakeConstants.DEPLOY_CANCODER_ID);

        spinMotor = new TalonFX(IntakeConstants.SPIN_MOTOR_ID);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.deployAngle = getDeployMotorAbsPos();
        inputs.spinMotorSetSpeed = getSpinMotorSpeed();

        inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
        inputs.deployMotorSuppliedCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
    }

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
}
