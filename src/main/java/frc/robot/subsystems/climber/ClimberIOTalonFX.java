// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.RealSubsystem;

/** Add your docs here. */
public class ClimberIOTalonFX extends RealSubsystem implements ClimberIO {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final PIDController controller = ClimberConstants.controller;

  private final PositionDutyCycle request = new PositionDutyCycle(0.0);

  public ClimberIOTalonFX() {

    leftMotor = new TalonFX(ClimberConstants.leftClimberMotorID);
    rightMotor = new TalonFX(ClimberConstants.leftClimberMotorID);

    configureHardware();
  }

  @Override
  protected void configureHardware() {

    leftMotor.getConfigurator().apply(ClimberConstants.climberTalonConfig);
    rightMotor.getConfigurator().apply(ClimberConstants.climberTalonConfig);
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

  @Override
  public void setClimbMotorSpeed(double speed) {
    leftMotor.setControl(new DutyCycleOut(speed));
    rightMotor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setTarget(Distance height) {
    leftMotor.setControl(request.withPosition(height.in(Meters)));
    rightMotor.setControl(request.withPosition(height.in(Meters)));
  }
}
