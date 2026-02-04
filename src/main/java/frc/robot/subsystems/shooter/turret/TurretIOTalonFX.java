// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID;
import frc.robot.RealSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;

/** Add your docs here. */
public class TurretIOTalonFX extends RealSubsystem implements TurretIO {

  // Motors
  private final TalonFX turretRotationMotor;

  private TurretParameters params = new TurretParameters();

  public TurretIOTalonFX() {
    // Initialize motors
    turretRotationMotor = new TalonFX(ShooterConstants.TURRET_ROTATION_ID, HardwareID.MAIN_CANBUS);
  }

  protected void configureHardware() {
    turretRotationMotor.getConfigurator().apply(ShooterConstants.TURRET_FEEDBACK_CONFIGS);
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
    inputs.turretAngleSuppliedVoltage.mut_replace(
        turretRotationMotor.getSupplyVoltage().getValue());
    inputs.turretAngleCurrentDraw.mut_replace(turretRotationMotor.getSupplyCurrent().getValue());
  }

  @Override
  public void setTurretMotorVolts(Voltage volts) {
    turretRotationMotor.setVoltage(volts.magnitude());
  }

  @Override
  public void zeroTurretEncoder() {
    turretRotationMotor.setPosition(Degrees.zero());
  }

  @Override
  public TurretParameters getTurretParameters() {
    Angle currentTurretAngle = getTurretAngularPosition();
    Angle targetTurretAngle = ShotCalculator.instance().getTargetTurretAngle();

    Angle error = currentTurretAngle.minus(targetTurretAngle);
    Angle clampedError = Degrees.of(error.in(Degrees) % 360);
    if (clampedError.in(Degrees) > 180) {
      params.turretRotationError = clampedError.minus(Degrees.of(360));
    } else if (clampedError.in(Degrees) < -180) {
      params.turretRotationError = clampedError.plus(Degrees.of(360));

    } else {
      params.turretRotationError = clampedError;
    }

    params.turretRotation = getTurretAngularPosition();

    return params;
  }

  // Helper functions
  private Angle getTurretAngularPosition() {
    return turretRotationMotor.getPosition().getValue();
  }
}
