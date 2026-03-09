// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretParameters;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private TurretIO turretIO;
  private TurretInputsAutoLogged loggedTurret = new TurretInputsAutoLogged();

  public Turret(TurretIO turretIO) {
    this.turretIO = turretIO;

    turretIO.zeroTurretEncoder();
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(loggedTurret);

    Logger.processInputs("Shooter/Turret", loggedTurret);

    RobotState.instance().setTurretRotationAngle(loggedTurret.turretMotorAngle);

    RobotState.instance().setTurretCurrentDraw(loggedTurret.turretMotorCurrent);
  }

  public TurretParameters getTurretParameters() {
    return turretIO.getTurretParameters();
  }

  public void setTurretRotationMotorVoltage(Voltage volts) {
    double maxVolts = ShooterConstants.MAX_TURRET_ROTATION_MOTOR_VOLTS.magnitude();
    turretIO.setTurretMotorVolts(Volts.of(MathUtil.clamp(volts.magnitude(), -maxVolts, maxVolts)));
  }

  public Command setTurretRotationVoltage(Voltage volts) {
    return Commands.runOnce(
        () -> {
          double maxVolts = ShooterConstants.MAX_TURRET_ROTATION_MOTOR_VOLTS.magnitude();
          turretIO.setTurretMotorVolts(
              Volts.of(MathUtil.clamp(volts.magnitude(), -maxVolts, maxVolts)));
        },
        this);
  }

  public void setTurretRotationMotorGains(Slot0Configs configs) {
    turretIO.updateTurretSlot0Configs(configs);
  }

  public void setTurretRotationMotorMM(MotionMagicConfigs configs) {
    turretIO.updateTurretMMConfigs(configs);
  }

  public void setTurretControlRequest(ControlRequest request) {
    turretIO.setTurretControlRequest(request);
  }

  public void zeroTurretPosition() {
    turretIO.zeroTurretEncoder();
  }

  public Command zeroTurretPositionCommand() {
    return Commands.runOnce(() -> turretIO.zeroTurretEncoder());
  }

  public Command setActuatorPosition(Distance position) {
    return Commands.runOnce(() -> turretIO.setHoodActuatorMM(position));
  }

  public void setActuatorPositionFunc(Distance position) {
    turretIO.setHoodActuatorMM(position);
  }

  public Current getTurretFFAmps() {
    if (turretIO
        .getTurretParameters()
        .turretRotation
        .gt(ShooterConstants.TURRET_SPRING_ANGLE_ZERO)) {
      double rot = turretIO.getTurretParameters().turretRotation.in(Radians);
      double value = 9.21 + (4.17 * rot) - (2.66 * Math.pow(rot, 2));
      return Amps.of(MathUtil.clamp(value, 0, 11.0));
    } else {
      double rot = turretIO.getTurretParameters().turretRotation.in(Radians);
      double value = 50.3 + (61.9 * rot) + (14.5 * Math.pow(rot, 2));
      return Amps.of(MathUtil.clamp(value, -15.5, 0));
    }
  }

  public boolean limitActivated() {
    return turretIO.limitActivated();
  }
}
