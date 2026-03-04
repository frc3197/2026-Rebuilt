// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

// Align command that drives robot to the target pose
public class AlignCommand extends Command {

  // Subsystems
  private final Align align;
  private final Drive drive;

  // Target to drive to
  private final Supplier<Pose2d> targetPose;

  private ChassisSpeeds speed = new ChassisSpeeds();

  public AlignCommand(Align align, Drive drive, Supplier<Pose2d> targetPose) {

    this.align = align;
    this.drive = drive;
    this.targetPose = targetPose;

    addRequirements(align, drive);
  }

  @Override
  public void execute() {
    speed =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            align.alignWithTarget(targetPose.get(), drive.getPose()), drive.getRotation());
    drive.runVelocity(speed);
  }

  @Override
  public boolean isFinished() {
    return Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) < 0.1
        && Math.abs(speed.omegaRadiansPerSecond) < 0.1;
  }
}
