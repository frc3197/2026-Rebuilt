// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LoggingConstants;
import frc.robot.enums.Modes.HoodMode;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretParameters;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class DefaultTurretHoodCommand extends Command {

  // Logged tunable gains
  private LoggedTunableNumber turretAngle_kP =
      new LoggedTunableNumber("turretAngle_kP", ShooterConstants.TURRET_SLOT0_CONFIGS.kP);
  private LoggedTunableNumber turretAngle_kI =
      new LoggedTunableNumber("turretAngle_kI", ShooterConstants.TURRET_SLOT0_CONFIGS.kI);
  private LoggedTunableNumber turretAngle_kD =
      new LoggedTunableNumber("turretAngle_kD", ShooterConstants.TURRET_SLOT0_CONFIGS.kD);
  private LoggedTunableNumber turretAngle_kS =
      new LoggedTunableNumber("turretAngle_kS", ShooterConstants.TURRET_SLOT0_CONFIGS.kS);
  private LoggedTunableNumber turretAngle_kV =
      new LoggedTunableNumber("turretAngle_kV", ShooterConstants.TURRET_SLOT0_CONFIGS.kV);
  private LoggedTunableNumber turretAcceleration =
      new LoggedTunableNumber(
          "turretAcceleration",
          ShooterConstants.TURRET_MOTOR_CONFIG.MotionMagic.MotionMagicAcceleration);
  private LoggedTunableNumber turretVelocity =
      new LoggedTunableNumber(
          "turretVelocity",
          ShooterConstants.TURRET_MOTOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity);

  // Arbitrary feed-forward to fight spring tension
  private LoggedTunableNumber turretFFProp =
      new LoggedTunableNumber("Turret CURRENT TORQUE", ShooterConstants.TURRET_SPRING_FF);

  private final Turret turret;
  private final DoubleSupplier turretMotorVoltageSupplier;

  /**
   * Creates a new DefaultFlywheelCommand.
   *
   * @param turret The turret subsystem.
   * @param turretMotorVoltageSupplier Supplies turret motor voltage magnitude, -1 to 1, maps to
   *     -10v to 10v.
   */
  public DefaultTurretHoodCommand(Turret turret, DoubleSupplier turretMotorVoltageSupplier) {
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
    if (LoggingConstants.tuningMode) {
      checkRotationGains();
    }

    // If the robot is not in shot calibration mode, then set the actuator to its
    // target extension
    if (!LoggingConstants.shooterCalibrationMode) {
      if (RobotState.instance().getHoodMode() == HoodMode.DOWN) {
        turret.setActuatorPositionFunc(Millimeters.of(0.0));
      } else {
        turret.setActuatorPositionFunc(ShotCalculator.instance().getTargetHoodExtension());
      }
    }

    // Update turret state
    switch (RobotState.instance().getTurretMode()) {
      case TRACKING_HUB:
        turretAutoTracking();
        // turret.setTurretControlRequest(new VelocityDutyCycle(0.2));
        // turret.setTurretControlRequest(new
        // TorqueCurrentFOC(Amps.of(turretFFProp.get())));
        break;

      case PASSING:
        turretAutoTracking();
        break;

        // Stop turret rotation
      case IDLE:
        ShooterConstants.TURRET_VOLTAGE_REQUEST.Output = 0.0;
        turret.setTurretControlRequest(ShooterConstants.TURRET_VOLTAGE_REQUEST);

        /*
         * turret.setTurretControlRequest(
         * ShooterConstants.TURRET_MOTION_MAGIC_REQUEST.withPosition(
         * ShooterConstants.TURRET_SPRING_ANGLE_ZERO));
         */
        break;

        // Rotate turret based on supplier, -3 to 3 volts
      case MANUAL:
        double manualVolts = turretMotorVoltageSupplier.getAsDouble() * 3;
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
        ShooterConstants.TURRET_MOTION_MAGIC_REQUEST
            .withPosition(params.turretRotationTarget)
            .withFeedForward(turret.getTurretFFAmps()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurretRotationMotorVoltage(Volts.of(0.0));
  }

  private void checkRotationGains() {
    if (turretAcceleration.hasChanged(hashCode())
        || turretVelocity.hasChanged(hashCode())
        || turretAngle_kP.hasChanged(hashCode())
        || turretAngle_kI.hasChanged(hashCode())
        || turretAngle_kD.hasChanged(hashCode())
        || turretAngle_kS.hasChanged(hashCode())
        || turretAngle_kV.hasChanged(hashCode())) {
      Slot0Configs newConfigs = new Slot0Configs();
      newConfigs.kP = turretAngle_kP.getAsDouble();
      newConfigs.kI = turretAngle_kI.getAsDouble();
      newConfigs.kD = turretAngle_kD.getAsDouble();
      newConfigs.kV = turretAngle_kV.getAsDouble();
      newConfigs.kS = turretAngle_kS.getAsDouble();

      MotionMagicConfigs newMM =
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(turretAcceleration.getAsDouble())
              .withMotionMagicCruiseVelocity(turretVelocity.getAsDouble());

      turret.setTurretRotationMotorMM(newMM);
      turret.setTurretRotationMotorGains(newConfigs);
    }
  }
}
