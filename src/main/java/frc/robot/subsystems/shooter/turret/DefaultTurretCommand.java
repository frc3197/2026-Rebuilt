// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretParameters;
import frc.robot.util.LoggedTunableNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultTurretCommand extends Command {

  private PIDController turretAnglePIDController = ShooterConstants.TURRET_ANGLE_PID_CONTROLLER;

  // Logged tunable gains
  private LoggedTunableNumber turretAngle_kP =
      new LoggedTunableNumber("turretAngle_kP", turretAnglePIDController.getP());
  private LoggedTunableNumber turretAngle_kI =
      new LoggedTunableNumber("turretAngle_kI", turretAnglePIDController.getI());
  private LoggedTunableNumber turretAngle_kD =
      new LoggedTunableNumber("turretAngle_kD", turretAnglePIDController.getD());

  private Turret turret;

  public DefaultTurretCommand(Turret turret) {
    this.turret = turret;
    addRequirements(this.turret);
  }

  @Override
  public void initialize() {
    this.turret.zeroTurretPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Check for gain updates
    if (turretAngle_kP.hasChanged(hashCode()))
      turretAnglePIDController.setP(turretAngle_kP.getAsDouble());
    if (turretAngle_kI.hasChanged(hashCode()))
      turretAnglePIDController.setI(turretAngle_kI.getAsDouble());
    if (turretAngle_kD.hasChanged(hashCode()))
      turretAnglePIDController.setD(turretAngle_kD.getAsDouble());

    TurretParameters params = turret.getTurretParameters();
    double calculatedVoltage =
        turretAnglePIDController.calculate(params.turretRotationError.in(Degrees));

    if (params.turretRotation.gt(ShooterConstants.TURRET_ROTATION_LIMIT_FORWARD)) {
      calculatedVoltage = calculatedVoltage > 0 ? 0 : calculatedVoltage;
    } else if (params.turretRotation.lt(ShooterConstants.TURRET_ROTATION_LIMIT_REVERSE)) {
      calculatedVoltage = calculatedVoltage < 0 ? 0 : calculatedVoltage;
    }

    turret.setTurretRotationMotorVoltage(Volts.of(calculatedVoltage));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurretRotationMotorVoltage(Volts.of(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
