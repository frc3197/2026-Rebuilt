// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.RealSubsystem;

/** Add your docs here. */
public class ClimberIOTalonFX extends RealSubsystem implements ClimberIO {

  // private final TalonFX climberMotor;

  public ClimberIOTalonFX() {
    // climberMotor = new TalonFX(ClimberConstants.climberMotor);

    // configureHardware();
  }

  @Override
  protected void configureHardware() {
    // climberMotor.getConfigurator().apply(ClimberConstants.climberTalonConfig);
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    // inputs.motorCurrent = climberMotor.getSupplyCurrent().getValueAsDouble();
    // inputs.climberAngleDegrees = climberMotor.getPosition().getValue().in(Degrees);
  }

  @Override
  public void setClimbMotorSpeed(double speed) {
    // climberMotor.set(speed);
  }

  @Override
  public void setClimberControl(ControlRequest request) {
    // climberMotor.setControl(request);
  }

  @Override
  public void zeroClimber() {
    // climberMotor.setPosition(0.0);
  }

  @Override
  public Angle getClimberAngle() {
    return Degrees.of(0.0);
    // return climberMotor.getPosition().getValue();
  }
}
