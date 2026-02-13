// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.RealSubsystem;

/** Add your docs here. */
public class ClimberIOTalonFX extends RealSubsystem implements ClimberIO {

  private final TalonFX climberMotor;

  private final PositionDutyCycle request = new PositionDutyCycle(0.0);

  public ClimberIOTalonFX() {

    climberMotor = new TalonFX(ClimberConstants.climberMotor);

    configureHardware();
  }

  @Override
  protected void configureHardware() {
    climberMotor.getConfigurator().apply(ClimberConstants.climberTalonConfig);
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.motorCurrent = climberMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setClimbMotorSpeed(double speed) {
    System.out.println("SPEED");
    climberMotor.set(speed);
  }
}
