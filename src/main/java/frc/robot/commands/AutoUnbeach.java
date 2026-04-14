// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoUnbeach extends Command {

  private final Drive drive;

  /** Creates a new AutoUnbeach. */
  public AutoUnbeach(Drive drive) {

    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(drive.getPose3d().getRotation().getMeasureX().in(Degrees)) > 12.0
        || Math.abs(drive.getPose3d().getRotation().getMeasureX().in(Degrees)) > 12.0) {
      drive.runVelocity(new ChassisSpeeds(-1.2, 0, 0));
    } else {
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
