// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RealSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Add your docs here. */
public class TurretIOTalonFX extends RealSubsystem implements TurretIO {

  // Encoders
  private final CANcoder turretRotationEncoder;

  // Motors
  private final TalonFX flywheelMotor;
  private final TalonFX turretRotationMotor;

  public TurretIOTalonFX() {
    // Initialize encoder
    turretRotationEncoder = new CANcoder(ShooterConstants.TURRET_ENCODER_ID);

    // Initialize motors
    flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID);
    turretRotationMotor = new TalonFX(ShooterConstants.TURRET_ROTATION_ID);
  }

  protected void configureHardware() {}

  @Override
  public void updateInputs(TurretInputs inputs) {
    inputs.turretAngleSuppliedVoltage = turretRotationMotor.getSupplyVoltage().getValueAsDouble();
    inputs.turretAngleSuppliedVoltage = turretRotationMotor.getSupplyVoltage().getValueAsDouble();
  }

  @Override
  public void setTurretMotorVolts(Voltage volts) {
    turretRotationMotor.setVoltage(volts.magnitude());
  }

  @Override
  public TurretParameters getTurretParameters() {
    return new TurretParameters();
  }
}
