// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.HardwareID;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.RealSubsystem;

/** Add your docs here. */
public class TurretIOTalonFX extends RealSubsystem implements TurretIO {

  // Motors
  private final TalonFX turretRotationMotor;

  private TurretParameters params = new TurretParameters();

  public TurretIOTalonFX() {
    // Initialize motors
    turretRotationMotor =
        new TalonFX(ShooterConstants.TURRET_ROTATION_ID, HardwareID.DRIVETRAIN_CANBUS);

    configureHardware();
  }

  protected void configureHardware() {
    turretRotationMotor.getConfigurator().apply(ShooterConstants.TURRET_MOTOR_CONFIG);
    turretRotationMotor.getConfigurator().apply(ShooterConstants.TURRET_SLOT0_CONFIGS);
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
    inputs.turretAngleSuppliedVoltage.mut_replace(
        turretRotationMotor.getSupplyVoltage().getValue());
    inputs.turretAngleCurrentDraw.mut_replace(turretRotationMotor.getSupplyCurrent().getValue());
    inputs.turretAngleSuppliedVoltage.mut_replace(
        turretRotationMotor.getSupplyVoltage().getValue());
    inputs.turretMotorAngle.mut_replace(getTurretAngularPosition());
  }

  @Override
  public void updateTurretSlot0Configs(Slot0Configs newConfig) {
    DriverStation.reportWarning("UPDATING TURRET GAINS", false);
    turretRotationMotor.getConfigurator().apply(newConfig);
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
  public void setTurretControlRequest(ControlRequest request) {
    turretRotationMotor.setControl(request);
  }

  @Override
  public void setOutputTargetAngle(Angle rotations) {
    ShooterConstants.TURRET_POSITION_REQUEST.Position = rotations.magnitude();
  }
  ;

  @Override
  public TurretParameters getTurretParameters() {
    Angle currentTurretAngle = getTurretAngularPosition();
    Angle targetTurretAngle = ShotCalculator.instance().getTargetTurretAngle();

    if (targetTurretAngle.gt(Degrees.of(180))) {
      targetTurretAngle = targetTurretAngle.minus(Degrees.of(360));
    }

    if (targetTurretAngle.lt(Degrees.of(-180))) {
      targetTurretAngle = targetTurretAngle.plus(Degrees.of(360));
    }

    params.turretRotationError = getTurretAngularPosition().minus(targetTurretAngle);
    params.turretRotation = getTurretAngularPosition();
    params.turretRotationTarget = targetTurretAngle;

    return params;
  }

  // Helper functions
  private Angle getTurretAngularPosition() {
    return turretRotationMotor.getPosition().getValue();
  }
}
