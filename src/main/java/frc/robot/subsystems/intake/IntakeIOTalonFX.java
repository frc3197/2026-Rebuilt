package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeIO;

public class IntakeIOTalonFX implements IntakeIO {
    
    private final TalonFX deployMotor;
    private final CANcoder deployCANcoder;

    private final TalonFX spinMotor;
    private final CANcoder spinCANcoder;
    

    public IntakeIOTalonFX() {
        deployMotor    = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID);
        deployCANcoder = new CANcoder(IntakeConstants.DEPLOY_CANCODER_ID);

        spinMotor      = new TalonFX(IntakeConstants.SPIN_MOTOR_ID);
        spinCANcoder   = new CANcoder(IntakeConstants.SPIN_CANCODER_ID);
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
    public void updateInputs(IntakeInputs inputs) {
        
    } 
}
