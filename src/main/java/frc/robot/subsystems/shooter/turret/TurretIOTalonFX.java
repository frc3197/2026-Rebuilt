// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.HardwareID;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LinearServo;
import frc.robot.util.RealSubsystem;

/** Add your docs here. */
public class TurretIOTalonFX extends RealSubsystem implements TurretIO {

  // Motors
  private final TalonFX turretRotationMotor;

  // Actuators
  private final LinearServo leftHoodActuator = new LinearServo(4, 50, 32);

  private DigitalInput turretZeroLimit = new DigitalInput(ShooterConstants.TURRET_ZERO_LIMIT);

  private TurretParameters params = new TurretParameters();

  private Distance hoodExtensionTarget = Millimeters.of(0.0);

  public TurretIOTalonFX() {
    // Initialize motors
    turretRotationMotor = new TalonFX(ShooterConstants.TURRET_ROTATION_ID, HardwareID.MAIN_CANBUS);

    configureHardware();
  }

  protected void configureHardware() {
    turretRotationMotor.getConfigurator().apply(ShooterConstants.TURRET_MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(TurretInputs inputs) {

    leftHoodActuator.updateCurPos();

    inputs.turretMotorAngle.mut_replace(getTurretAngularPosition());
    inputs.turretMotorCurrent.mut_replace(turretRotationMotor.getSupplyCurrent().getValue());
    inputs.turretMotorVoltage.mut_replace(turretRotationMotor.getMotorVoltage().getValue());
    inputs.hoodAngle.mut_replace(getTurretAngularPosition());
    inputs.hoodActuatorExtension.mut_replace(Millimeters.of(leftHoodActuator.getPosition()));
    inputs.hoodActuatorExtensionTarget.mut_replace(hoodExtensionTarget);
    inputs.limitSwitchActivated = limitActivated();
  }

  @Override
  public void updateTurretSlot0Configs(Slot0Configs newConfig) {
    DriverStation.reportWarning("UPDATING TURRET GAINS", false);
    turretRotationMotor.getConfigurator().apply(newConfig);
  }

  @Override
  public void setHoodActuatorMM(Distance distance) {
    hoodExtensionTarget = distance;
    leftHoodActuator.setPosition(distance.in(Millimeters));
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
  public void updateTurretMMConfigs(MotionMagicConfigs request) {
    turretRotationMotor.getConfigurator().apply(request);
  }

  @Override
  public void setOutputTargetAngle(Angle rotations) {
    ShooterConstants.TURRET_POSITION_REQUEST.Position = rotations.magnitude();
  }

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

    params.turretRotationError.mut_replace(currentTurretAngle.minus(targetTurretAngle));
    params.turretRotation.mut_replace(currentTurretAngle);
    params.turretRotationTarget.mut_replace(targetTurretAngle);

    params.hoodActuatorExtension.mut_replace(Millimeters.of(leftHoodActuator.getPosition()));
    params.hoodActuatorExtensionTarget.mut_replace(hoodExtensionTarget);
    params.hoodAngle.mut_replace(getTurretHoodAngleFromPosition(leftHoodActuator.getPosition()));

    return params;
  }

  @Override
  public boolean limitActivated() {
    return !turretZeroLimit.get();
  }

  // Helper functions
  private Angle getTurretAngularPosition() {
    return turretRotationMotor.getPosition().getValue();
  }

  private Angle getTurretHoodAngleFromPosition(double position) {
    return Degrees.of(
        MathUtil.interpolate(
            ShooterConstants.MIN_HOOD_ANGLE.in(Degrees),
            ShooterConstants.MAX_HOOD_ANGLE.in(Degrees),
            position / 55));
  }
}
