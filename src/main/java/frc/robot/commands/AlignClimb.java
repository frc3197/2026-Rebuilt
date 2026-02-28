// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignClimb extends Command {
  /** Creates a new AlignClimb. */
  private final DoubleSupplier horizontalOffsetSupplier;

  private final Drive drive;

  private final double maxSpeed = 0.35;

  public AlignClimb(DoubleSupplier horizontalOffsetSupplier, Drive drive) {
    this.horizontalOffsetSupplier = horizontalOffsetSupplier;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("CLIMB ALIGN OFFSET", horizontalOffsetSupplier.getAsDouble());
    drive.runVelocity(
        new ChassisSpeeds(
            MathUtil.clamp(
                MathUtil.applyDeadband(
                    -VisionConstants.CLIMB_ALIGN_PID_CONTROLLER.calculate(
                        horizontalOffsetSupplier.getAsDouble()),
                    0.02),
                -maxSpeed,
                maxSpeed),
            0.0,
            0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(horizontalOffsetSupplier.getAsDouble())
        < VisionConstants.CLIMB_ALIGNED_THRESHOLD.in(Millimeters);
  }
}
