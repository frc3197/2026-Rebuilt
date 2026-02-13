// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretParameters;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimTurretCommand extends Command {

  private PIDController turretAnglePIDController = new PIDController(0.5, 0.0, 0.0);
  private final DoubleSupplier turretAngleMotorVoltageManual;

  private Turret turret;

  public SimTurretCommand(Turret turret, DoubleSupplier angleVoltageManual) {
    this.turret = turret;
    this.turretAngleMotorVoltageManual = angleVoltageManual;
    addRequirements(this.turret);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Simulate the control requests because CTRE does not have a good sim method
    switch (RobotState.instance().getTurretMode()) {
      case TRACKING_HUB:
        simTrackPeriodic();
        break;

      case PASSING:
        simTrackPeriodic();
        break;

      case IDLE:
        turret.setTurretRotationMotorVoltage(Volts.of(0.0));
        break;

      case MANUAL:
        turret.setTurretRotationMotorVoltage(Volts.of(turretAngleMotorVoltageManual.getAsDouble()));
        break;

      default:
        break;
    }
  }

  // Helper
  private void simTrackPeriodic() {
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
}
