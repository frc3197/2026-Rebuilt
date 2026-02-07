// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretParameters;
import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {

  private TurretIO turretIO;

  private TurretInputsAutoLogged loggedShooter = new TurretInputsAutoLogged();

  public Turret(TurretIO turretIO) {
    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(loggedShooter);

    RobotState.instance().setTurretRotationAngle(loggedShooter.turretMotorAngle);
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

  public Command setTurretRotationVoltage(DoubleSupplier value) {
    return Commands.run(
        () -> {
          turretIO.setTurretMotorVolts(
              Volts.of(
                  ShooterConstants.MAX_TURRET_ROTATION_MOTOR_VOLTS.magnitude()
                      * value.getAsDouble()));
        },
        this);
  }

  public void zeroTurretPosition() {
    turretIO.zeroTurretEncoder();
  }
}
