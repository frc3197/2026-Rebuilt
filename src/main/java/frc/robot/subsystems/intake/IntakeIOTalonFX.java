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
    private final CANcoder spinCANcoder;
    

    public IntakeIOTalonFX() {
        deployMotor    = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID);
        deployCANcoder = new CANcoder(IntakeConstants.DEPLOY_CANCODER_ID);

        spinMotor      = new TalonFX(IntakeConstants.SPIN_MOTOR_ID);
        spinCANcoder   = new CANcoder(IntakeConstants.SPIN_CANCODER_ID);
    }

    public Angle GetSpinMotorAbsPos() {
        return spinCANcoder.getAbsolutePosition().getValue();
    }

    public Angle GetDeployMotorAbsPos() {
        return deployCANcoder.getAbsolutePosition().getValue();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        
    } 
}
