// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LoggingConstants;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretParameters;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class DefaultTurretCommand extends Command {

  // Logged tunable gains
  private LoggedTunableNumber turretAngle_kP =
      new LoggedTunableNumber("turretAngle_kP", ShooterConstants.TURRET_SLOT0_CONFIGS.kP);
  private LoggedTunableNumber turretAngle_kI =
      new LoggedTunableNumber("turretAngle_kI", ShooterConstants.TURRET_SLOT0_CONFIGS.kI);
  private LoggedTunableNumber turretAngle_kD =
      new LoggedTunableNumber("turretAngle_kD", ShooterConstants.TURRET_SLOT0_CONFIGS.kD);

  private final Turret turret;
  private final DoubleSupplier turretMotorVoltageSupplier;

  /**
   * Creates a new DefaultFlywheelCommand.
   *
   * @param turret The turret subsystem.
   * @param turretMotorVoltageSupplier Supplies turret motor voltage magnitude.
   */
  public DefaultTurretCommand(Turret turret, DoubleSupplier turretMotorVoltageSupplier) {
    this.turretMotorVoltageSupplier = turretMotorVoltageSupplier;
    this.turret = turret;
    addRequirements(this.turret);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Check for gain updates
    if (LoggingConstants.tuningMode
            && (turretAngle_kP.hasChanged(hashCode()) || turretAngle_kI.hasChanged(hashCode()))
        || turretAngle_kD.hasChanged(hashCode())) {
      Slot0Configs newConfigs = new Slot0Configs();
      newConfigs.kP = turretAngle_kP.getAsDouble();
      newConfigs.kI = turretAngle_kI.getAsDouble();
      newConfigs.kD = turretAngle_kD.getAsDouble();
      turret.setTurretRotationMotorGains(newConfigs);
    }

    switch (RobotState.instance().getTurretMode()) {
      case TRACKING_HUB:
        // turretAutoTracking();
        break;

      case PASSING:
        // turretAutoTracking();
        break;

      case IDLE:
        ShooterConstants.TURRET_VOLTAGE_REQUEST.Output = 0.0;
        turret.setTurretControlRequest(ShooterConstants.TURRET_VOLTAGE_REQUEST);
        break;

      case MANUAL:
        double manualVolts = turretMotorVoltageSupplier.getAsDouble() * 3;
        /*
        TurretParameters params = turret.getTurretParameters();
        if (params.turretRotation.gt(ShooterConstants.TURRET_ROTATION_LIMIT_FORWARD)) {
          manualVolts =
              turretMotorVoltageSupplier.getAsDouble() > 0
                  ? 0
                  : turretMotorVoltageSupplier.getAsDouble();
        } else if (params.turretRotation.lt(ShooterConstants.TURRET_ROTATION_LIMIT_REVERSE)) {
          manualVolts =
              turretMotorVoltageSupplier.getAsDouble() < 0
                  ? 0
                  : turretMotorVoltageSupplier.getAsDouble();
        }*/
        turret.setTurretControlRequest(
            ShooterConstants.TURRET_VOLTAGE_REQUEST.withOutput(Volts.of(manualVolts)));

        break;

      default:
        break;
    }
  }

  // Helper methods
  private void turretAutoTracking() {
    TurretParameters params = turret.getTurretParameters();

    turret.setTurretControlRequest(
        ShooterConstants.TURRET_MOTION_MAGIC_REQUEST.withPosition(params.turretRotationTarget));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurretRotationMotorVoltage(Volts.of(0.0));
  }
}
