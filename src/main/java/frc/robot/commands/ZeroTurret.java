// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.subsystems.shooter.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroTurret extends Command {
  /** Creates a new ZeroTurret. */
  private final Turret turret;

  private double factor = 1.0;

  public ZeroTurret(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PAHFKJF");
    if (turret.getTurretParameters().turretRotation.in(Degrees) > 0) {
      factor = -1.0;
    } else {
      factor = 1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setTurretControlRequest(new VoltageOut(1.0 * factor));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurretControlRequest(new VoltageOut(0.0));
    RobotContainer.setTurretMode(TurretMode.IDLE);
    if (turret.limitActivated()) {
      turret.zeroTurretPosition();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.limitActivated();
  }
}
