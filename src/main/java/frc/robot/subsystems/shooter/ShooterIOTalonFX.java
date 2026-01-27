// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RealSubsystem;

/** Add your docs here. */
public class ShooterIOTalonFX extends RealSubsystem implements ShooterIO {

  // Encoders
  private final CANcoder turretRotationEncoder;

  // Motors
  private final TalonFX flywheelMotor;
  private final TalonFX turretRotationMotor;

  public ShooterIOTalonFX() {

    // Initialize encoder
    turretRotationEncoder = new CANcoder(ShooterConstants.TURRET_ENCODER_ID);

    // Initialize motors
    flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID);
    turretRotationMotor = new TalonFX(ShooterConstants.TURRET_ROTATION_ID);
  }

  protected void configureHardware() {}

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.turretSuppliedVoltage = turretRotationMotor.getSupplyVoltage().getValueAsDouble();
    inputs.turretSuppliedVoltage = turretRotationMotor.getSupplyVoltage().getValueAsDouble();
  }
}
