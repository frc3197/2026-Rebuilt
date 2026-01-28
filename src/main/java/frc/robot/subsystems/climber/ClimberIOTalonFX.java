// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public ClimberIOTalonFX() {
        leftMotor = new TalonFX(ClimberConstants.leftClimberMotorID);
        rightMotor = new TalonFX(ClimberConstants.leftClimberMotorID);
    }

    public void setLeftClimberMotorSpeed(double speed) {
        leftMotor.set(speed);
    }

    public void setRightClimberMotorSpeed(double speed) {
        rightMotor.set(speed);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.leftMotorCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rightMotorCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
    }

}
